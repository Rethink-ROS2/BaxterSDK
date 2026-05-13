"""Unit tests for the baxter_interface.analog_io module."""
import errno
import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from baxter_core_msgs.msg import AnalogIOState, AnalogOutputCommand
from baxter_interface.analog_io import AnalogIO

STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

COMPONENT_ID = 'test_id'
STATE_TOPIC = f'/robot/analog_io/{COMPONENT_ID}/state'
COMMAND_TOPIC = '/robot/analog_io/command'


class TestAnalogIO(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.executor = MultiThreadedExecutor()
        cls.executor_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.executor_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_analog_io')
        self.pub_node = Node('test_state_publisher')
        self.executor.add_node(self.node)
        self.executor.add_node(self.pub_node)
        self._state_pub = self.pub_node.create_publisher(AnalogIOState, STATE_TOPIC, STATE_QOS)

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.pub_node)
        self.node.destroy_node()
        self.pub_node.destroy_node()

    def _publish_state(self, value, is_input_only, delay=0.1):
        msg = AnalogIOState()
        msg.value = value
        msg.is_input_only = is_input_only

        def _do_publish():
            time.sleep(delay)
            self._state_pub.publish(msg)

        threading.Thread(target=_do_publish, daemon=True).start()

    def test_input_state_received(self):
        self._publish_state(value=42, is_input_only=True)
        io = AnalogIO(self.node, COMPONENT_ID)
        self.assertEqual(io.state(), 42)
        self.assertFalse(io.is_output())

    def test_output_state_received(self):
        self._publish_state(value=100, is_input_only=False)
        io = AnalogIO(self.node, COMPONENT_ID)
        self.assertEqual(io.state(), 100)
        self.assertTrue(io.is_output())

    def test_timeout_raises_when_no_state_published(self):
        with self.assertRaises(OSError) as ctx:
            AnalogIO(self.node, COMPONENT_ID)
        self.assertEqual(ctx.exception.errno, errno.ETIMEDOUT)

    def test_set_output_raises_when_input_only(self):
        self._publish_state(value=0, is_input_only=True)
        io = AnalogIO(self.node, COMPONENT_ID)
        with self.assertRaises(IOError):
            io.set_output(42)

    def test_set_output_publishes_command(self):
        received = []

        def on_command(msg):
            received.append(msg.value)
            self._publish_state(value=msg.value, is_input_only=False, delay=0.05)

        self.node.create_subscription(AnalogOutputCommand, COMMAND_TOPIC, on_command, 10)

        self._publish_state(value=0, is_input_only=False)
        io = AnalogIO(self.node, COMPONENT_ID)
        io.set_output(42)

        self.assertIn(42, received)


if __name__ == '__main__':
    unittest.main()
