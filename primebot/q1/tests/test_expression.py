"""No hardware playback in these tests."""
import threading
import unittest
from unittest.mock import Mock
from expression import EMOTIONS, ExpressionCard, playback_spec, interpret_response


class ExpressionContracts(unittest.TestCase):
    def test_catalog_and_single_play_request(self):
        self.assertEqual(len(EMOTIONS), 33)
        for name, number in EMOTIONS.items():
            spec = playback_spec("play", {"emotion": name})
            self.assertEqual(spec["emotion_ids"], [number])
            self.assertEqual(spec["loop_count"], 1)
            self.assertEqual(spec["priority"], 0)
            self.assertEqual(spec["type"], "emotion")
            self.assertEqual(spec["file_paths"], [])

    def test_reset_means_calm_not_device_reset(self):
        self.assertEqual(playback_spec("reset", {})["emotion_ids"], [3001])

    def test_invalid_selection_rejected_before_transport(self):
        for value in (None, "", "跳舞", 3001, [], {}):
            with self.assertRaises(ValueError):
                playback_spec("play", {"emotion": value})

    def card(self):
        card = ExpressionCard.__new__(ExpressionCard)
        card.lock = threading.Lock()
        card.closed = threading.Event()
        card._request = Mock(return_value={"state": "accepted"})
        return card

    def test_lifecycle_does_not_call_robot(self):
        card = self.card()
        card.start()
        for action in ("start", "stop"):
            self.assertFalse(card.dispatch(action, {})["request_sent"])
        card._request.assert_not_called()

    def test_play_and_info_use_separate_operations(self):
        card = self.card()
        card.dispatch("info", {})
        card._request.assert_called_once_with("info", {})
        card._request.reset_mock()
        card.dispatch("play", {"emotion": "开心"})
        card._request.assert_called_once_with("play", {"emotion": "开心"})

    def test_busy_and_closed_do_not_send(self):
        card = self.card()
        card.lock.acquire()
        self.assertEqual(card.dispatch("reset", {})["state"], "busy")
        card.lock.release()
        card.stop()
        self.assertEqual(card.dispatch("reset", {})["state"], "error")
        card._request.assert_not_called()

    def test_both_response_codes_required_and_no_false_completion(self):
        spec = playback_spec("play", {"emotion": "开心"})
        self.assertTrue(interpret_response(0, 1, "", spec)["accepted"])
        for code, status in [(1, 1), (0, 2), (0, 0)]:
            result = interpret_response(code, status, "拒绝", spec)
            self.assertFalse(result["accepted"])
            self.assertIsNone(result["playback_completed"])


if __name__ == "__main__":
    unittest.main()
