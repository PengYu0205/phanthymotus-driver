"""Read-only Q1 joint telemetry and model for Motus's skeleton renderer."""

import json
import math
import multiprocessing
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from threading import Lock

INTERVAL = 0.05
STALE_AFTER = 1.0
SOURCE = "/aima/hal/joint/state"
MODEL = Path(__file__).with_name("q1_v3.urdf")
IMU_SOURCE = "/aima/hal/imu/state"
IMU_STALE_AFTER = 0.5


def decode_orientation(msg):
    """Return a normalized body-to-reference quaternion, in renderer wxyz order."""
    if len(msg.orientation_covariance) and msg.orientation_covariance[0] == -1:
        return None
    q = [getattr(msg.orientation, name) for name in ("w", "x", "y", "z")]
    if not all(math.isfinite(v) for v in q):
        return None
    norm = math.hypot(*q)
    if norm < 1e-8:
        return None
    return [v / norm for v in q]


def attach_orientation(data, now):
    """Keep attitude freshness independent from the joint stream."""
    result = dict(data)
    sample = result.pop("_attitude", None)
    age = max(0, now - sample["received"]) if sample else None
    valid = bool(sample and sample["quat"] is not None and age <= IMU_STALE_AFTER)
    result["imu_quat"] = sample["quat"] if valid else None
    result["imu_source"] = IMU_SOURCE
    result["imu_age_s"] = round(age, 3) if age is not None else None
    result["imu_state"] = "live" if valid else "stale" if sample else "waiting"
    return result


def model_joint_names(path=MODEL):
    root = ET.parse(path).getroot()
    return [j.attrib["name"] for j in root.findall("joint")
            if j.attrib.get("type") in ("revolute", "continuous", "prismatic")]


def decode_joints(msg, expected):
    """Match by name, never by the vendor array's changing order."""
    by_name = {}
    for joint in msg.joints:
        if joint.name in by_name:
            raise ValueError("Duplicate joint: " + joint.name)
        values = (joint.position, joint.velocity, joint.effort)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Non-finite joint sample: " + joint.name)
        by_name[joint.name] = joint
    missing = [name for name in expected if name not in by_name]
    unknown = sorted(set(by_name) - set(expected))
    if missing or unknown:
        raise ValueError(f"Joint/model mismatch: missing={missing}, unknown={unknown}")
    joints = []
    for idx, name in enumerate(expected):
        j = by_name[name]
        joints.append({"idx": idx, "name": name, "q": j.position,
                       "dq": j.velocity, "tau": j.effort,
                       "coil_temperature_c": j.coil_temp,
                       "motor_temperature_c": j.motor_temp})
    return {"joints": joints,
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
            "frame_id": msg.header.frame_id}


def read_joints(domain, output, stopping, expected):
    from common import logsafe
    logsafe.install(check_fd=False)
    from robot import configure_robot_dds
    import rclpy
    from aimdk_msgs.msg import JointStateArray
    from sensor_msgs.msg import Imu
    from rclpy.qos import qos_profile_sensor_data

    configure_robot_dds()
    rclpy.init(domain_id=domain)
    node = rclpy.create_node("primebot_q1_joints_reader", start_parameter_services=False)
    pending = None
    attitude = None

    def capture_imu(msg):
        nonlocal attitude
        attitude = {"received": time.monotonic(), "quat": decode_orientation(msg)}

    def capture(msg):
        nonlocal pending
        received = time.monotonic()
        try:
            pending = (received, decode_joints(msg, expected), None)
        except ValueError as exc:
            pending = (received, None, str(exc))

    def flush():
        nonlocal pending
        if pending is not None:
            received, data, error = pending
            if data is not None:
                data = {**data, "_attitude": attitude}
            output.send((received, data, error))
            pending = None

    node.create_subscription(JointStateArray, SOURCE, capture, qos_profile_sensor_data)
    # Q1's on-device legged odometry uses the raw IMU with pelvis as its body
    # reference. Do not substitute torso orientation: it includes waist motion.
    node.create_subscription(Imu, IMU_SOURCE, capture_imu, qos_profile_sensor_data)
    node.create_timer(INTERVAL, flush)
    try:
        while not stopping.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        output.close()
        node.destroy_node()
        rclpy.try_shutdown()


class JointsCard:
    def __init__(self, namespace, ros, robot_domain):
        from rclpy.node import Node
        from std_msgs.msg import String

        self.message_type = String
        self.expected = model_joint_names()
        self.topic = f"/{namespace}/state/joints"
        self.output = [{"topic": self.topic, "format": "sensor/skeleton"}]
        self.node = Node("primebot_q1_joints", context=ros.ctx_core)
        self.publisher = self.node.create_publisher(String, self.topic, 5)
        ros.executor_core.add_node(self.node)
        ctx = multiprocessing.get_context("spawn")
        self.receiver, self.sender = ctx.Pipe(duplex=False)
        self.stopping = ctx.Event()
        self.reader = ctx.Process(target=read_joints,
                                  args=(robot_domain, self.sender, self.stopping, self.expected))
        self.lock = Lock()
        self.sample = None
        self.error = None
        self.enabled = True
        self.timer = None

    def get_tools(self):
        from common.vendor_runtime import action_schema, tool

        return [tool("joints", "sensor", "启元 Q1 实时关节与 3D 骨架，只读，角度单位 rad。",
                     action_schema({"info": ([], "关节状态与数据话题")}, {}),
                     topic_out=self.output),
                tool("model", "resource", "启元 Q1 v3 骨架 URDF 模型。",
                     {"type": "object", "properties": {"topic": {"type": "string"}}})]

    def start(self):
        self.reader.start()
        self.sender.close()
        self.timer = self.node.create_timer(INTERVAL, self._tick)

    def stop(self):
        if self.stopping.is_set():
            return
        self.stopping.set()
        if self.timer:
            self.timer.cancel()
        if self.reader.pid:
            self.reader.join(timeout=3)
            if self.reader.is_alive():
                self.reader.terminate()
                self.reader.join(timeout=1)
        self.sender.close()
        with self.lock:
            self.receiver.close()
            self.node.destroy_node()

    def _payload(self):
        now = time.monotonic()
        age = max(0, now - self.sample[0]) if self.sample else None
        state = "waiting" if self.sample is None else "live" if age <= STALE_AFTER else "stale"
        if self.error:
            state = "error"
        if self.reader.pid and not self.reader.is_alive() and not self.stopping.is_set():
            state = "error"
        data = attach_orientation(self.sample[1], now) if self.sample and state == "live" else {
            "joints": [], "imu_quat": None, "imu_state": "waiting"}
        return {**data, "state": state, "stale": state != "live",
                "age_s": round(age, 3) if age is not None else None,
                "timestamp_ms": time.time_ns() // 1_000_000,
                "source": SOURCE, "position_unit": "rad",
                "expected_count": len(self.expected), "error": self.error}

    def _tick(self):
        with self.lock:
            if self.stopping.is_set():
                return
            while not self.receiver.closed and self.receiver.poll():
                try:
                    received, data, error = self.receiver.recv()
                except EOFError:
                    self.receiver.close()
                    self.error = "Joint reader exited"
                    break
                self.error = error
                if data is not None:
                    self.sample = (received, data)
            if self.enabled:
                self.publisher.publish(self.message_type(data=json.dumps(self._payload(), allow_nan=False)))

    def dispatch(self, action, args):
        if action == "model":
            return {"urdf": MODEL.read_text(), "name": "q1_v3", "position_unit": "rad"}
        if action not in ("start", "stop", "info", "status", "joints"):
            return None
        with self.lock:
            if action in ("start", "stop"):
                self.enabled = action == "start"
            return {"state": "running" if self.enabled else "idle", "topic_out": self.output,
                    "data": self._payload()}
