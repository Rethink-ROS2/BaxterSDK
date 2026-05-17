from abc import ABC, abstractmethod

import baxter_dataflow
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node


class BaxterNode(Node, ABC):
    """Base class for all Baxter tool scripts.

    Extends rclpy.node.Node directly so all ROS 2 node APIs are available
    on self: self.get_logger(), self.get_clock(), self.create_timer(), etc.

    Subclasses implement run(). execute() is the single entry point that
    owns the full ROS lifecycle: init, construct, run, destroy, shutdown.

    Example::

        class TuckArms(BaxterNode):
            def __init__(self):
                super().__init__('tuck_arms')
                self._limb = Limb(self, 'left')

            def run(self):
                while rclpy.ok():
                    self._limb.set_joint_positions(targets)
                    self.spin_rate(20)

        if __name__ == '__main__':
            TuckArms.execute()
    """

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

    def spin_rate(self, rate_hz: float) -> None:
        """Spin the executor for one period at the given rate.

        Use this at the bottom of every rate-limited loop to ensure
        subscription callbacks are delivered between iterations.
        """
        rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)

    @abstractmethod
    def run(self) -> None:
        """Execute the tool logic. Called by execute() after node init."""
        pass

    def on_shutdown(self) -> None:
        """Optional hook called after run() completes or raises.

        Override to release resources, disable the robot, or publish
        a final state before rclpy.shutdown() is called.
        """
        pass

    @classmethod
    def execute(cls, ros_args=None, **kwargs) -> None:
        """Template Method: initialise ROS, create the instance, run, shut down.

        rclpy.init() is called to initialises a global context.

        Args:
            ros_args: ROS 2 arguments. Defaults to None
        """
        rclpy.init(args=ros_args)
        instance = cls(**kwargs)
        try:
            instance.run()
        finally:
            instance.on_shutdown()
            instance.destroy_node()
            rclpy.shutdown()


class BaxterInterface:
    """Base class for all Baxter hardware interface wrappers.

    Provides a dedicated MutuallyExclusiveCallbackGroup, a consistent
    subscription factory, and a blocking wait helper.

    Example::

        class Head(BaxterInterface):
            def __init__(self, node: Node):
                super().__init__(node)
                self._state = None
                self._sub = self._subscribe(
                    HeadState, 'robot/head/head_state',
                    self._on_state, SENSOR_QOS,
                )
                self._wait_for(
                    lambda: self._state is not None,
                    timeout_msg='Failed to get head state',
                )

            def _on_state(self, msg: HeadState) -> None:
                self._state = msg
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._cb_group = MutuallyExclusiveCallbackGroup()

    def _subscribe(self, msg_type, topic: str, callback, qos):
        """Create a subscription bound to this interface's callback group."""
        return self._node.create_subscription(
            msg_type,
            topic,
            callback,
            qos,
            callback_group=self._cb_group,
        )

    def _wait_for(
        self,
        condition,
        timeout: float = 5.0,
        timeout_msg: str = 'timeout expired',
        body=None,
    ) -> None:
        """Block until condition() returns True, spinning the executor each iteration.

        Args:
            condition:   Zero-argument callable that returns True when satisfied.
            timeout:     Maximum seconds to wait. Negative means wait indefinitely.
            timeout_msg: Message attached to the TimeoutError if exceeded.
            body:        Optional callable invoked each iteration (e.g. re-publish a command).
        """
        baxter_dataflow.wait_for(
            self._node,
            condition,
            timeout=timeout,
            timeout_msg=timeout_msg,
            body=body,
        )
