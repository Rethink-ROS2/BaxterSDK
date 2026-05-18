Architecture Patterns
=====================

This page defines the design patterns used throughout the workspace.
All new code and all ports from ROS 1 follow these patterns.

.. contents::
   :local:
   :depth: 2


Tool Entry Point
----------------

Every command-line tool inherits from ``BaxterNode``.
The ``execute()`` classmethod handles ``rclpy.init()``, runs the tool,
and calls ``rclpy.shutdown()`` unconditionally.

.. code-block:: python

    from baxter_interface.base import BaxterNode

    class EnableRobot(BaxterNode):
        def __init__(self, enable: bool):
            super().__init__('enable_robot')
            self._rs = RobotEnable(self.node)
            self._enable = enable

        def run(self):
            if self._enable:
                self._rs.enable(self.node)
            else:
                self._rs.disable(self.node)

    def main():
        EnableRobot.execute(enable=True)


Rate-Limited Loops
------------------

``spin_rate(rate_hz)`` replaces ``rate.sleep()`` at the bottom of every
timed loop. It calls ``rclpy.spin_once()`` for one period, which delivers
pending subscription callbacks before the next iteration.

.. code-block:: python

    def run(self):
        while rclpy.ok():
            self._limb.set_joint_positions(self._targets)
            self.spin_rate(20)   # 20 Hz; delivers callbacks each cycle


Hardware Interface Classes
--------------------------

Every hardware wrapper inherits from ``BaxterInterface``.
Subscriptions are registered through ``_subscribe()``, which binds them
to the instance's ``MutuallyExclusiveCallbackGroup``.
Initial state is always awaited with ``_wait_for()`` before ``__init__`` returns.

.. code-block:: python

    from baxter_interface.base import BaxterInterface

    class Limb(BaxterInterface):
        def __init__(self, node: Node, limb: str):
            super().__init__(node)
            self.name = limb
            self._joint_angle: dict[str, float] = {}

            self._joint_state_sub = self._subscribe(
                JointState, 'robot/joint_states',
                self._on_joint_states, SENSOR_QOS,
            )
            self._wait_for(
                lambda: len(self._joint_angle) > 0,
                timeout_msg=f'{limb} limb failed to get joint states',
            )

        def _on_joint_states(self, msg: JointState) -> None:
            for idx, name in enumerate(msg.name):
                if name in self._joint_names[self.name]:
                    self._joint_angle[name] = msg.position[idx]


Blocking Commands
-----------------

When a command must be retried until the robot confirms it, use ``_wait_for``
with a ``body`` argument. The body is called on every iteration alongside
``rclpy.spin_once()``, so the confirmation callback is delivered without
a background thread.

.. code-block:: python

    def _toggle_enabled(self, status: bool) -> None:
        self._wait_for(
            condition=lambda: self._state.enabled == status,
            timeout=2.0,
            timeout_msg=f'Failed to {"en" if status else "dis"}able robot',
            body=lambda: self._enable_pub.publish(Bool(data=status)),
        )


Publishers
----------

Publishers are instance variables created in ``__init__``, never inside methods.

.. code-block:: python

    class RobotEnable(BaxterInterface):
        def __init__(self, node: Node):
            super().__init__(node)
            self._state = None
            self._state_sub = self._subscribe(
                AssemblyState, 'robot/state', self._on_state, LATCH_QOS,
            )
            self._enable_pub  = node.create_publisher(Bool,  'robot/set_super_enable', LATCH_QOS)
            self._reset_pub   = node.create_publisher(Empty, 'robot/set_super_reset',  LATCH_QOS)
            self._stop_pub    = node.create_publisher(Empty, 'robot/set_super_stop',   LATCH_QOS)
            self._wait_for(lambda: self._state is not None,
                           timeout_msg='Failed to get robot state')


QoS Profiles
------------

Two profiles cover all Baxter topics. Define them as module-level constants
and reuse them; do not construct ``QoSProfile`` inline.

.. code-block:: python

    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    LATCH_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    SENSOR_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

Use ``LATCH_QOS`` for state and command topics that a late-joining subscriber
must receive immediately (robot state, enable/disable commands).
Use ``SENSOR_QOS`` for high-frequency sensor streams where dropped messages
are acceptable (joint states, endpoint states, collision state).


Logging
-------

Use the node logger at the appropriate level. Never use ``print()``.
Never log inside high-frequency callbacks such as ``_on_joint_states``.

.. code-block:: python

    self._node.get_logger().info('Robot enabled')
    self._node.get_logger().warn('Non-fatal error on reset; check diagnostics')
    self._node.get_logger().error('E-Stop asserted; disengage before continuing')

For anything called faster than 10 Hz, use throttled logging:

.. code-block:: python

    self._node.get_logger().info_throttle(1.0, 'Still waiting for joint states...')


Message Construction
--------------------

Always construct the ROS message type explicitly. Never pass bare Python
primitives to ``publish()``.

.. code-block:: python

    self._enable_pub.publish(Bool(data=True))
    self._speed_pub.publish(Float64(data=0.3))
    self._reset_pub.publish(Empty())
