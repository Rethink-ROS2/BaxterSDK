#!/usr/bin/env python3

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

"""Baxter RSDK Joint Position Example: joystick"""

import argparse
import sys

import rclpy
from rclpy.utilities import remove_ros_args

import baxter_external_devices
import baxter_interface
from baxter_interface import BaxterNode


def rotate(lst):
    """Rotates a list left."""
    if len(lst):
        v = lst[0]
        lst[:-1] = lst[1:]
        lst[-1] = v


def set_j(cmd, limb, joints, index, delta):
    """Set the selected joint to current pos + delta."""
    joint = joints[index]
    cmd[joint] = delta + limb.joint_angle(joint)


class JointPositionJoystick(BaxterNode):
    """RSDK Joint Position Example: Joystick Control

    Use a game controller to control the angular joint positions
    of Baxter's arms.

    Attach a game controller to your dev machine and run this
    example along with the ROS joy_node to control the position
    of each joint in Baxter's arms using the joysticks. Be sure to
    provide your *joystick* type to setup appropriate key bindings.

    Each stick axis maps to a joint angle; which joints are currently
    controlled can be incremented by using the mapped increment buttons.
    Ex:
      (x,y -> e0,e1) >>increment>> (x,y -> e1,e2)
    """

    def __init__(self, joystick_type: str):
        super().__init__('rsdk_joint_position_joystick')
        self._left = baxter_interface.Limb(self, 'left')
        self._right = baxter_interface.Limb(self, 'right')
        self._grip_left = baxter_interface.Gripper('left', False, self)
        self._grip_right = baxter_interface.Gripper('right', False, self)
        self._rs = baxter_interface.RobotEnable(self)

        if joystick_type == 'xbox':
            self._joystick = baxter_external_devices.joystick.XboxController(self)
        elif joystick_type == 'logitech':
            self._joystick = baxter_external_devices.joystick.LogitechController(self)
        elif joystick_type == 'ps3':
            self._joystick = baxter_external_devices.joystick.PS3Controller(self)

    def run(self):
        self.get_logger().info('Getting robot state...')
        self._init_state = self._rs.state().enabled

        self.get_logger().info('Enabling robot...')
        self._rs.enable(self)

        self._map_joystick()
        self.get_logger().info('Done.')

    def on_shutdown(self):
        if not self._init_state:
            self.get_logger().info('Disabling robot...')
            self._rs.disable(self)

    def _map_joystick(self):
        """Maps joystick input to joint position commands."""
        left = self._left
        right = self._right
        grip_left = self._grip_left
        grip_right = self._grip_right
        lcmd = {}
        rcmd = {}

        lj = left.joint_names()
        rj = right.joint_names()

        def jhi(s):
            return self._joystick.stick_value(s) > 0

        def jlo(s):
            return self._joystick.stick_value(s) < 0

        bdn = self._joystick.button_down
        bup = self._joystick.button_up

        def print_help(bindings_list):
            print('Press Ctrl-C to quit.')
            for bindings in bindings_list:
                for test, _cmd, doc in bindings:
                    if callable(doc):
                        doc = doc()
                    print('%s: %s' % (str(test[1][0]), doc))

        bindings_list = []
        bindings = (
            ((bdn, ['rightTrigger']), (grip_left.close, []), 'left gripper close'),
            ((bup, ['rightTrigger']), (grip_left.open, []), 'left gripper open'),
            ((bdn, ['leftTrigger']), (grip_right.close, []), 'right gripper close'),
            ((bup, ['leftTrigger']), (grip_right.open, []), 'right gripper open'),
            ((jlo, ['leftStickHorz']), (set_j, [rcmd, right, rj, 0, 0.1]), lambda: 'right inc ' + rj[0]),
            ((jhi, ['leftStickHorz']), (set_j, [rcmd, right, rj, 0, -0.1]), lambda: 'right dec ' + rj[0]),
            ((jlo, ['rightStickHorz']), (set_j, [lcmd, left, lj, 0, 0.1]), lambda: 'left inc ' + lj[0]),
            ((jhi, ['rightStickHorz']), (set_j, [lcmd, left, lj, 0, -0.1]), lambda: 'left dec ' + lj[0]),
            ((jlo, ['leftStickVert']), (set_j, [rcmd, right, rj, 1, 0.1]), lambda: 'right inc ' + rj[1]),
            ((jhi, ['leftStickVert']), (set_j, [rcmd, right, rj, 1, -0.1]), lambda: 'right dec ' + rj[1]),
            ((jlo, ['rightStickVert']), (set_j, [lcmd, left, lj, 1, 0.1]), lambda: 'left inc ' + lj[1]),
            ((jhi, ['rightStickVert']), (set_j, [lcmd, left, lj, 1, -0.1]), lambda: 'left dec ' + lj[1]),
            ((bdn, ['rightBumper']), (rotate, [lj]), 'left: cycle joint'),
            ((bdn, ['leftBumper']), (rotate, [rj]), 'right: cycle joint'),
            ((bdn, ['btnRight']), (grip_left.calibrate, []), 'left calibrate'),
            ((bdn, ['btnLeft']), (grip_right.calibrate, []), 'right calibrate'),
            ((bdn, ['function1']), (print_help, [bindings_list]), 'help'),
            ((bdn, ['function2']), (print_help, [bindings_list]), 'help'),
        )
        bindings_list.append(bindings)

        print_help(bindings_list)
        print('Press Ctrl-C to stop.')
        while rclpy.ok():
            for test, cmd, doc in bindings:
                if test[0](*test[1]):
                    cmd[0](*cmd[1])
                    if callable(doc):
                        print(doc())
                    else:
                        print(doc)
            if len(lcmd):
                left.set_joint_positions(lcmd)
                lcmd.clear()
            if len(rcmd):
                right.set_joint_positions(rcmd)
                rcmd.clear()
            self.spin_rate(100)


def main():
    epilog = """
See help inside the example with the "Start" button for controller key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        formatter_class=arg_fmt,
        description=JointPositionJoystick.__doc__,
        epilog=epilog,
    )
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-j',
        '--joystick',
        required=True,
        choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use',
    )
    args = parser.parse_args(remove_ros_args(sys.argv[1:]))
    JointPositionJoystick.execute(joystick_type=args.joystick)


if __name__ == '__main__':
    main()
