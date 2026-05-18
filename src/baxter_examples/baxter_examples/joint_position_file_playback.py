#!/usr/bin/env python

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

"""
Baxter RSDK Joint Position Example: file playback
"""

from __future__ import print_function

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

import baxter_interface


class JointPosePlayback:
    def __init__(self, node: Node):
        self._node = node

    @staticmethod
    def try_float(x):
        try:
            return float(x)
        except ValueError:
            return None

    def clean_line(self, line, names):
        """
        Cleans a single line of recorded joint positions

        @param line: the line described in a list to process
        @param names: joint name keys
        """
        # convert the line of strings to a float or None
        line = [self.try_float(x) for x in line.rstrip().split(',')]
        # zip the values with the joint names
        combined = zip(names[1:], line[1:])
        # take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        # convert it to a dictionary with only valid commands
        command = dict(cleaned)
        left_command = dict((key, command[key]) for key in command.keys() if key[:-2] == 'left_')
        right_command = dict((key, command[key]) for key in command.keys() if key[:-2] == 'right_')
        return (command, left_command, right_command, line)

    def map_file(self, filename, loops=1):
        """
        Loops through csv file

        @param filename: the file to play
        @param loops: number of times to loop
                    values < 0 mean 'infinite'

        Does not loop indefinitely, but only until the file is read
        and processed. Reads each line, split up in columns and
        formats each line into a controller command in the form of
        name/value pairs. Names come from the column headers
        first column is the time stamp
        """
        left = baxter_interface.Limb('left', node=self._node)
        right = baxter_interface.Limb('right', node=self._node)
        grip_left = baxter_interface.Gripper('left', node=self._node)
        grip_right = baxter_interface.Gripper('right', node=self._node)

        if grip_left.error():
            grip_left.reset()
        if grip_right.error():
            grip_right.reset()
        if not grip_left.calibrated() and grip_left.type() != 'custom':
            grip_left.calibrate()
        if not grip_right.calibrated() and grip_right.type() != 'custom':
            grip_right.calibrate()

        print('Playing back: %s' % (filename,))
        with open(filename, 'r') as f:
            lines = f.readlines()
        keys = lines[0].rstrip().split(',')

        loop_count = 0
        # If specified, repeat the file playback 'loops' number of times
        while loops < 1 or loop_count < loops:
            i = 0
            loop_count += 1
            print('Moving to start position...')

            _cmd, lcmd_start, rcmd_start, _raw = self.clean_line(lines[1], keys)
            self._node.get_logger().info('Calling move_to_joint_positions for start...')
            left.move_to_joint_positions(lcmd_start)
            right.move_to_joint_positions(rcmd_start)
            self._node.get_logger().info('move_to_joint_positions returned')

            ros_before = self._node.get_clock().now().nanoseconds / 1e9
            wall_before = time.time()
            time.sleep(0.1)
            ros_after = self._node.get_clock().now().nanoseconds / 1e9
            wall_after = time.time()
            self._node.get_logger().info(
                'Clock check: wall_elapsed=%.6f ros_elapsed=%.6f' % (wall_after - wall_before, ros_after - ros_before)
            )

            start_time = self._node.get_clock().now().nanoseconds / 1e9
            self._node.get_logger().info(
                'start_time=%.3f first_ts=%.6f last_ts=%.6f total_records=%d'
                % (start_time, float(lines[1].split(',')[0]), float(lines[-1].split(',')[0]), len(lines) - 1)
            )

            for values in lines[1:]:
                i += 1
                loopstr = str(loops) if loops > 0 else 'forever'
                sys.stdout.write('\r Record %d of %d, loop %d of %s' % (i, len(lines) - 1, loop_count, loopstr))
                sys.stdout.flush()

                cmd, lcmd, rcmd, values = self.clean_line(values, keys)
                elapsed_at_entry = self._node.get_clock().now().nanoseconds / 1e9 - start_time
                iters = 0
                # command this set of commands until the next frame
                while (self._node.get_clock().now().nanoseconds / 1e9 - start_time) < values[0]:
                    iters += 1
                    if not rclpy.ok():
                        print('\n Aborting - ROS shutdown')
                        return False
                    if len(lcmd):
                        left.set_joint_positions(lcmd)
                    if len(rcmd):
                        right.set_joint_positions(rcmd)
                    if 'left_gripper' in cmd and grip_left.type() != 'custom' and grip_left.calibrated():
                        grip_left.command_position(cmd['left_gripper'])
                    if 'right_gripper' in cmd and grip_right.type() != 'custom' and grip_right.calibrated():
                        grip_right.command_position(cmd['right_gripper'])
                    rclpy.spin_once(self._node, timeout_sec=0)
                    time.sleep(0.001)
                if i <= 5 or iters > 0:
                    self._node.get_logger().info(
                        'rec %d: ts=%.4f elapsed_entry=%.4f iters=%d' % (i, values[0], elapsed_at_entry, iters)
                    )
            print()
        return True


def main():
    """RSDK Joint Position Example: File Playback

    Uses Joint Position Control mode to play back a series of
    recorded joint and gripper positions.

    Run the joint_recorder.py example first to create a recording
    file for use with this example. This example uses position
    control to replay the recorded positions in sequence.

    Note: This version of the playback example simply drives the
    joints towards the next position at each time stamp. Because
    it uses Position Control it will not attempt to adjust the
    movement speed to hit set points "on time".
    """
    epilog = """
Related examples:
  joint_recorder.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__, epilog=epilog)
    parser.add_argument('-f', '--file', metavar='PATH', required=True, help='path to input file')
    parser.add_argument(
        '-l', '--loops', type=int, default=1, help='number of times to loop the input file. 0=infinite.'
    )
    args = parser.parse_args()

    print('Initializing node... ')
    rclpy.init()
    node = rclpy.create_node('rsdk_joint_position_file_playback')
    print('Getting robot state... ')
    rs = baxter_interface.RobotEnable(node)
    init_state = rs.state().enabled

    def clean_shutdown():
        print('\nExiting example...')
        if not init_state:
            print('Disabling robot...')
            rs.disable(node)

    rclpy.get_default_context().on_shutdown(clean_shutdown)

    print('Enabling robot... ')
    rs.enable(node)

    playback = JointPosePlayback(node)
    playback.map_file(args.file, args.loops)


if __name__ == '__main__':
    main()
