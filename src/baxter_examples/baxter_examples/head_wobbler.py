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

from __future__ import print_function

import argparse
import random
import time

import rclpy
from rclpy.node import Node

import baxter_interface


class Wobbler(object):
    def __init__(self, node: Node):
        """
        'Wobbles' the head
        """
        self._done = False
        self._node = node
        self._head = baxter_interface.Head(self._node)

        # verify robot is enabled
        print('Getting robot state... ')
        self._rs = baxter_interface.RobotEnable(self._node)
        self._init_state = self._rs.state().enabled
        print('Enabling robot... ')
        self._rs.enable(self._node)
        print('Running. Ctrl-c to quit')

    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        disabling the robot
        """
        print('\nExiting example...')
        try:
            self.set_neutral()
        except BaseException:
            pass
        if self._rs.state().enabled:
            print('Disabling robot...')
            self._rs.disable(self._node)

    def set_neutral(self):
        """
        Sets the head back into a neutral pose
        """
        self._head.set_pan(0.0, timeout=0)

    def wobble(self):
        """
        Performs the wobbling
        """
        self.set_neutral()
        self._head.command_nod()
        command_rate = self._node.create_rate(1)
        control_rate = self._node.create_rate(100)
        start = time.time()
        while rclpy.ok() and (time.time() - start < 10.0):
            angle = random.uniform(-1.5, 1.5)
            while rclpy.ok() and not (abs(self._head.pan() - angle) <= baxter_interface.HEAD_PAN_ANGLE_TOLERANCE):
                self._head.set_pan(angle, speed=0.3, timeout=0)
                control_rate.sleep()
            command_rate.sleep()

        self._done = True
        rclpy.shutdown()


def main():
    """RSDK Head Example: Wobbler

    Nods the head and pans side-to-side towards random angles.
    Demonstrates the use of the baxter_interface.Head class.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    parser.parse_args()

    print('Initializing node... ')
    rclpy.init()
    node = rclpy.create_node('rsdk_head_wobbler')

    wobbler = Wobbler(node)
    print('Wobbling... ')
    try:
        wobbler.wobble()
    finally:
        wobbler.clean_shutdown()
    print('Done.')


if __name__ == '__main__':
    main()
