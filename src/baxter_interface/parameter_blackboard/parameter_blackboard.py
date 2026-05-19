# ROS2 port: replaces dynamic_reconfigure.Server with ROS2 node parameters.
# Parameters are declared on the node at construction time and can be
# changed at runtime via `ros2 param set`.

from rclpy.node import Node

_JOINTS = (
    'left_s0',
    'left_s1',
    'left_e0',
    'left_e1',
    'left_w0',
    'left_w1',
    'left_w2',
    'right_s0',
    'right_s1',
    'right_e0',
    'right_e1',
    'right_w0',
    'right_w1',
    'right_w2',
)

# Per-mode defaults matching the original .cfg files
_DEFAULTS = {
    'position_w_id': {
        'goal_time': 0.1,
        'stopped_velocity_tolerance': 0.20,
        '_goal': -1.0,
        '_trajectory': 0.35,
    },
    'position': {
        'goal_time': 0.1,
        'stopped_velocity_tolerance': 0.25,
        '_goal': -1.0,
        '_trajectory': 0.20,
    },
    'velocity': {
        'goal_time': 0.0,
        'stopped_velocity_tolerance': -1.0,
        '_goal': -1.0,
        '_trajectory': -1.0,
        '_kp': 2.0,
        '_ki': 0.0,
        '_kd': 0.0,
    },
}


class _ConfigProxy:
    """Dict-like view over node parameters; mirrors dyn.config['key'] access."""

    def __init__(self, node: Node):
        self._node = node

    def __getitem__(self, key: str):
        return self._node.get_parameter(key).value


class ParameterBlackboard:
    """
    Replaces dynamic_reconfigure.Server for the trajectory action server.

    Declares all controller parameters on the given node so they can be
    read via self.config['param_name'] and set at runtime with
    `ros2 param set <node> <param> <value>`.
    """

    def __init__(self, node: Node, mode: str):
        self._node = node
        if mode not in _DEFAULTS:
            raise ValueError('Unknown mode %r; expected one of %s' % (mode, list(_DEFAULTS)))
        self._declare_parameters(mode)

    def _declare_parameters(self, mode: str):
        defaults = _DEFAULTS[mode]
        self._node.declare_parameter('goal_time', defaults['goal_time'])
        self._node.declare_parameter('stopped_velocity_tolerance', defaults['stopped_velocity_tolerance'])
        for joint in _JOINTS:
            self._node.declare_parameter(joint + '_goal', defaults['_goal'])
            self._node.declare_parameter(joint + '_trajectory', defaults['_trajectory'])
            if mode == 'velocity':
                self._node.declare_parameter(joint + '_kp', defaults['_kp'])
                self._node.declare_parameter(joint + '_ki', defaults['_ki'])
                self._node.declare_parameter(joint + '_kd', defaults['_kd'])

    @property
    def config(self) -> _ConfigProxy:
        return _ConfigProxy(self._node)
