"""Q1 built-in facial expressions. Lifecycle actions never change the screen."""
import multiprocessing
import threading
import time

EMOTIONS = {
    "平静": 3001, "耍酷": 3002, "开心": 3003, "好奇": 3004, "困惑": 3005,
    "调皮": 3006, "崇拜": 3007, "撒娇": 3008, "困倦": 3009, "眨眼": 3010,
    "闭眼": 3011, "左右环顾": 3012, "好奇-2": 3013, "沮丧": 3014, "害羞": 3015,
    "生气": 3016, "无语": 3017, "难过": 3018, "震惊": 3019, "睁眼": 3020,
    "断网": 3021, "空表情": 3022, "静音": 3023, "思考": 3025, "聆听": 3026,
    "说话": 3027, "低电量": 3028, "极低电量": 3029, "充电中": 3030,
    "识别扫描": 3035, "拍照录像": 3036, "QQ音乐听歌": 3045, "喜马拉雅听书": 3046,
}
SERVICE = "/aimdk_5Fmsgs/srv/PlayEmotion"


def playback_spec(action, args):
    if action not in ("play", "reset"):
        raise ValueError("未知表情动作")
    emotion = "平静" if action == "reset" else args.get("emotion")
    if not isinstance(emotion, str) or emotion not in EMOTIONS:
        raise ValueError("请选择有效的内置表情")
    return {"type": "emotion", "emotion_ids": [EMOTIONS[emotion]], "file_paths": [],
            "priority": 0, "loop_count": 1, "emotion": emotion}


def interpret_response(code, status, message, spec):
    accepted = code == 0 and status == 1
    return {"state": "accepted" if accepted else "rejected", "accepted": accepted,
            "playback_completed": None, "request_sent": True,
            "emotion": spec["emotion"], "emotion_id": spec["emotion_ids"][0],
            "response_code": code, "response_status": status,
            "message": message or ("服务已接受播放请求；未确认屏幕播放完成" if accepted else "表情请求被拒绝")}


def expression_request(domain, action, args, output, cancelled, sent, deadline):
    from common import logsafe
    logsafe.install(check_fd=False)
    import rclpy
    from robot import configure_robot_dds
    from aimdk_msgs.srv import PlayEmotion

    node = None
    try:
        spec = playback_spec(action, args) if action != "info" else None
        configure_robot_dds()
        rclpy.init(domain_id=domain)
        node = rclpy.create_node("q1_expression_request", start_parameter_services=False)
        client = node.create_client(PlayEmotion, SERVICE)

        def check():
            if cancelled.is_set() or time.monotonic() >= deadline:
                raise TimeoutError("表情请求超时或取消；不自动重试")

        discovery_end = min(deadline, time.monotonic() + 3)
        while not client.service_is_ready() and time.monotonic() < discovery_end:
            check()
            rclpy.spin_once(node, timeout_sec=.05)
        ready = client.service_is_ready()
        if action == "info":
            result = {"state": "ready" if ready else "unavailable", "service_ready": ready,
                      "service": SERVICE, "request_sent": False,
                      "message": "查询服务，不改变屏幕", "emotions": EMOTIONS}
        elif not ready:
            result = {"state": "unavailable", "accepted": False, "request_sent": False,
                      "message": "PlayEmotion 服务不可用，未发送播放请求"}
        else:
            check()
            request = PlayEmotion.Request()
            request.header.header.stamp = node.get_clock().now().to_msg()
            for name in ("type", "emotion_ids", "file_paths", "priority", "loop_count"):
                setattr(request, name, spec[name])
            check()
            sent.set()
            future = client.call_async(request)
            try:
                while not future.done():
                    check()
                    rclpy.spin_once(node, timeout_sec=.05)
                response = future.result()
                if response is None:
                    raise RuntimeError("表情服务返回空响应，播放结果未知")
                common = response.header
                result = interpret_response(common.header.code, common.status.value, common.message, spec)
            finally:
                if not future.done():
                    future.cancel()
                    client.remove_pending_request(future)
        output.send(result)
    except Exception as exc:
        output.send({"state": "unknown" if sent.is_set() else "error", "request_sent": sent.is_set(),
                     "accepted": None if sent.is_set() else False, "message": str(exc)})
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        output.close()


class ExpressionCard:
    def __init__(self, namespace, ros, robot_domain):
        self.domain = robot_domain
        self.lock = threading.Lock()
        self.closed = threading.Event()
        self.latest = {"state": "idle", "request_sent": False, "message": "等待查询或播放"}

    def get_tool(self):
        from common.vendor_runtime import action_schema, tool
        return tool("expression", "actuator", "Q1 内置屏幕表情。播放一次或恢复平静，不驱动身体，不自动播放。",
                    action_schema({"info": ([], "查询表情服务及支持列表，不改变屏幕"),
                                   "play": (["emotion"], "播放所选内置表情一次"),
                                   "reset": ([], "播放平静表情一次，不是硬件复位")},
                                  {"emotion": {"type": "string", "enum": list(EMOTIONS),
                                               "description": "内置表情名称；状态类表情仅改变显示，不启动对应功能"}}))

    def start(self):
        pass

    def stop(self):
        self.closed.set()

    def _request(self, action, args):
        ctx = multiprocessing.get_context("spawn")
        receiver, sender = ctx.Pipe(duplex=False)
        cancelled, sent = ctx.Event(), ctx.Event()
        deadline = time.monotonic() + 8
        process = ctx.Process(target=expression_request,
                              args=(self.domain, action, args, sender, cancelled, sent, deadline))
        try:
            process.start()
            sender.close()
            while time.monotonic() < deadline and not self.closed.is_set():
                if receiver.poll(.05):
                    return receiver.recv()
                if not process.is_alive():
                    break
            return {"state": "unknown" if sent.is_set() else "error", "request_sent": sent.is_set(),
                    "accepted": None if sent.is_set() else False,
                    "message": "请求取消、进程退出或超时；不自动重试，已发送请求的播放结果未知"}
        except (EOFError, OSError) as exc:
            return {"state": "unknown" if sent.is_set() else "error", "request_sent": sent.is_set(),
                    "message": str(exc)}
        finally:
            cancelled.set()
            if process.pid:
                process.join(timeout=.5)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=1)
            receiver.close()
            sender.close()

    def dispatch(self, action, args):
        if action in ("start", "stop"):
            return {"state": "idle", "request_sent": False,
                    "message": "画布生命周期不播放、不清空表情；需要恢复平静时选择 reset"}
        if action not in ("info", "play", "reset"):
            return None
        if action != "info":
            playback_spec(action, args)
        if self.closed.is_set():
            return {"state": "error", "request_sent": False, "message": "表情驱动已关闭"}
        if not self.lock.acquire(blocking=False):
            return {"state": "busy", "request_sent": False, "message": "正在处理另一个表情请求"}
        try:
            self.latest = self._request(action, args)
            return self.latest
        finally:
            self.lock.release()
