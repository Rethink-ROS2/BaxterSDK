"""Unit tests for the baxter_interface.analog_io module."""

import errno
import unittest
from unittest.mock import MagicMock, patch

from baxter_core_msgs.msg import AnalogIOState

from baxter_interface.analog_io import AnalogIO

COMPONENT_ID = 'test_id'
STATE_TOPIC = f'/robot/analog_io/{COMPONENT_ID}/state'
COMMAND_TOPIC = '/robot/analog_io/command'


def _make_state_msg(value, is_input_only):
    msg = AnalogIOState()
    msg.value = value
    msg.is_input_only = is_input_only
    return msg


def _make_node():
    """Returns (mock_node, trigger_callback) where trigger_callback(msg) fires _on_io_state."""
    node = MagicMock()
    captured = {}

    def capture_subscription(msg_type, topic, callback, qos):
        captured['callback'] = callback
        return MagicMock()

    node.create_subscription.side_effect = capture_subscription

    def trigger(msg):
        captured['callback'](msg)

    return node, trigger


class TestAnalogIO(unittest.TestCase):
    def _make_io(self, node, trigger, value=0, is_input_only=True):
        """Construct AnalogIO, firing the state callback inside wait_for."""
        with patch('baxter_dataflow.wait_for') as mock_wait:

            def deliver_state(n, test, **kwargs):
                trigger(_make_state_msg(value, is_input_only))
                return True

            mock_wait.side_effect = deliver_state
            io = AnalogIO(node, COMPONENT_ID)
        return io

    def test_input_state_received(self):
        node, trigger = _make_node()
        io = self._make_io(node, trigger, value=42, is_input_only=True)
        self.assertEqual(io.state(), 42)
        self.assertFalse(io.is_output())

    def test_output_state_received(self):
        node, trigger = _make_node()
        io = self._make_io(node, trigger, value=100, is_input_only=False)
        self.assertEqual(io.state(), 100)
        self.assertTrue(io.is_output())

    def test_timeout_raises(self):
        node, _ = _make_node()
        with patch('baxter_dataflow.wait_for') as mock_wait:
            mock_wait.side_effect = OSError(errno.ETIMEDOUT, 'timeout')
            with self.assertRaises(OSError) as ctx:
                AnalogIO(node, COMPONENT_ID)
        self.assertEqual(ctx.exception.errno, errno.ETIMEDOUT)

    def test_set_output_raises_when_input_only(self):
        node, trigger = _make_node()
        io = self._make_io(node, trigger, value=0, is_input_only=True)
        with self.assertRaises(IOError):
            io.set_output(42)

    def test_set_output_publishes_command(self):
        node, trigger = _make_node()
        io = self._make_io(node, trigger, value=0, is_input_only=False)

        with patch('baxter_dataflow.wait_for') as mock_wait:

            def deliver_feedback(n, test, **kwargs):
                trigger(_make_state_msg(42, False))
                return True

            mock_wait.side_effect = deliver_feedback
            io.set_output(42)

        pub = node.create_publisher.return_value
        published_cmd = pub.publish.call_args[0][0]
        self.assertEqual(published_cmd.value, 42)
        self.assertEqual(published_cmd.name, COMPONENT_ID)


if __name__ == '__main__':
    unittest.main()
