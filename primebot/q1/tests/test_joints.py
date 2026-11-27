"""Contract tests for live telemetry mapping and stale-frame handling."""
import math
import time
import unittest
from types import SimpleNamespace as NS

from joints import JointsCard, decode_joints, model_joint_names, decode_orientation, attach_orientation


def message(names):
    return NS(header=NS(stamp=NS(sec=12, nanosec=34), frame_id="q1_joint_state"),
              joints=[NS(name=name, position=i * .01, velocity=.1, effort=.2,
                         coil_temp=40, motor_temp=45) for i, name in enumerate(names)])


class JointContracts(unittest.TestCase):
    def test_orientation_order_normalization_and_invalid_data(self):
        msg = NS(orientation=NS(w=2., x=0., y=2., z=0.), orientation_covariance=[0.] * 9)
        q = decode_orientation(msg)
        self.assertAlmostEqual(q[0], math.sqrt(.5))
        self.assertAlmostEqual(q[2], math.sqrt(.5))
        msg.orientation_covariance[0] = -1
        self.assertIsNone(decode_orientation(msg))
        msg.orientation_covariance[0] = 0
        msg.orientation.w = math.nan
        self.assertIsNone(decode_orientation(msg))
        msg.orientation = NS(w=0., x=0., y=0., z=0.)
        self.assertIsNone(decode_orientation(msg))

    def test_orientation_expires_independently_of_live_joints(self):
        sample = {"joints": [1], "_attitude": {"received": 5., "quat": [1., 0., 0., 0.]}}
        self.assertEqual(attach_orientation(sample, 5.1)["imu_state"], "live")
        result = attach_orientation(sample, 6.)
        self.assertIsNone(result["imu_quat"])
        self.assertEqual(result["joints"], [1])
        self.assertNotIn("_attitude", result)
        self.assertIn("_attitude", sample)

    def test_real_model_has_22_unique_movable_joints(self):
        names = model_joint_names()
        self.assertEqual(len(names), 22)
        self.assertEqual(len(set(names)), 22)

    def test_reordered_vendor_array_maps_by_name(self):
        names = model_joint_names()
        msg = message(list(reversed(names)))
        result = decode_joints(msg, names)
        self.assertEqual([j["name"] for j in result["joints"]], names)
        self.assertEqual(result["joints"][0]["q"], .21)

    def test_rejects_missing_duplicate_unknown_and_nonfinite(self):
        names = model_joint_names()
        for sample in [message(names[:-1]), message(names + [names[0]]),
                       message(names + ["unknown"] )]:
            with self.assertRaises(ValueError):
                decode_joints(sample, names)
        for value in [math.nan, math.inf]:
            sample = message(names)
            sample.joints[0].position = value
            with self.assertRaises(ValueError):
                decode_joints(sample, names)

    def test_stale_and_invalid_samples_never_animate_as_live(self):
        card = JointsCard.__new__(JointsCard)
        card.expected = model_joint_names()
        card.error = None
        card.reader = NS(pid=None)
        card.sample = None
        self.assertEqual(card._payload()["state"], "waiting")
        card.sample = (time.monotonic(), decode_joints(message(card.expected), card.expected))
        self.assertEqual(len(card._payload()["joints"]), 22)
        card.sample = (time.monotonic() - 2, card.sample[1])
        self.assertEqual(card._payload()["state"], "stale")
        self.assertEqual(card._payload()["joints"], [])
        card.error = "Invalid frame"
        self.assertEqual(card._payload()["state"], "error")


if __name__ == "__main__":
    unittest.main()
