# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import errno

import baxter_dataflow
from baxter_core_msgs.msg import AssemblyState
from rclpy.node import Node
from std_msgs.msg import Bool as Bool
from std_msgs.msg import Empty as Empty


class RobotEnable(object):
    """
    Class RobotEnable - simple control/status wrapper around robot state

    enable()  - enable all joints
    disable() - disable all joints
    reset()   - reset all joints, reset all jrcp faults, disable the robot
    stop()    - stop the robot, similar to hitting the e-stop button
    """

    def __init__(self, node: Node):
        """
        Constructor for RobotEnable class.
        """
        self._state = None
        state_topic = 'robot/state'
        self._state_sub = node.create_subscription(AssemblyState, state_topic, self._state_callback, 10)

        baxter_dataflow.wait_for(
            node,
            lambda: self._state is not None,
            timeout=2.0,
            timeout_msg=(f'Failed to get robot state on {state_topic}'),
        )

    def _state_callback(self, msg):
        self._state = msg

    def _toggle_enabled(self, node: Node, status):
        pub = node.create_publisher(Bool, 'robot/set_super_enable', 10)

        baxter_dataflow.wait_for(
            node,
            test=lambda: self._state.enabled == status,
            timeout=2.0 if status else 5.0,
            timeout_msg=(f'Failed to {"en" if status else "dis"}able robot'),
            body=lambda: pub.publish(Bool(data=status)),
        )

        node.get_logger().info(f'Robot {"Enabled" if status else "Disabled"}')

    def state(self):
        """
        Returns the last known robot state.
        """
        return self._state

    def enable(self, node: Node):
        """
        Enable all joints
        """
        if self._state.stopped:
            node.get_logger().info('Robot Stopped: Attempting Reset...')
            self.reset(node)
        self._toggle_enabled(node=node, status=True)

    def disable(self, node: Node):
        """
        Disable all joints
        """
        self._toggle_enabled(node=node, status=False)

    def reset(self, node: Node):
        """
        Reset all joints.  Trigger JRCP hardware to reset all faults.  Disable
        the robot.
        """
        error_estop = """\
E-Stop is ASSERTED. Disengage E-Stop and then reset the robot.
"""
        error_nonfatal = """Non-fatal Robot Error on reset.
Robot reset cleared stopped state and robot can be enabled, but a non-fatal
error persists. Check diagnostics or rethink.log for more info.
"""
        error_env = """Failed to reset robot.
Please verify that the ROS_IP or ROS_HOSTNAME environment variables are set
and resolvable.
"""

        def is_reset():
            return (
                not self._state.enabled
                and not self._state.stopped
                and not self._state.error
                and self._state.estop_button == 0
                and self._state.estop_source == 0
            )

        pub = node.create_publisher(Empty, 'robot/set_super_reset', 10)

        if self._state.stopped and self._state.estop_button == AssemblyState.ESTOP_BUTTON_PRESSED:
            node.get_logger().error(error_estop)
            raise IOError(errno.EREMOTEIO, 'Failed to Reset: E-Stop Engaged')

        node.get_logger().info('Resetting robot...')
        try:
            baxter_dataflow.wait_for(node, test=is_reset, timeout=3.0, timeout_msg=error_env, body=pub.publish)
        except OSError as e:
            if e.errno == errno.ETIMEDOUT:
                if self._state.error and not self._state.stopped:
                    node.get_logger().warn(error_nonfatal)
                    return False
            raise

    def stop(self, node: Node):
        """
        Simulate an e-stop button being pressed.  Robot must be reset to clear
        the stopped state.
        """
        pub = node.create_publisher(Empty, 'robot/set_super_stop', 10)
        baxter_dataflow.wait_for(
            node,
            test=lambda: self._state.stopped,
            timeout=3.0,
            timeout_msg='Failed to stop the robot',
            body=pub.publish,
        )
        node.get_logger().info('Robot Stopped')
