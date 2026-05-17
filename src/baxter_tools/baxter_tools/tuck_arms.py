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

"""
Tool to tuck/untuck Baxter's arms to/from the shipping pose
"""

import argparse
from copy import deepcopy

import rclpy
from baxter_core_msgs.msg import (
    CollisionAvoidanceState,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import (
    Bool,
    Empty,
)

import baxter_interface


class Tuck(object):
    def __init__(self, node: Node, tuck_cmd):
        self._done = False
        self._limbs = ('left', 'right')
        self._arms = {
            'left': baxter_interface.Limb(node, 'left'),
            'right': baxter_interface.Limb(node, 'right'),
        }
        self._node = node
        self._tuck = tuck_cmd
        self._tuck_rate = node.create_rate(20.0)  # Hz
        self._tuck_threshold = 0.2  # radians
        self._peak_angle = -1.6  # radians
        self._arm_state = {
            'tuck': {'left': 'none', 'right': 'none'},
            'collide': {'left': False, 'right': False},
            'flipped': {'left': False, 'right': False},
        }
        self._joint_moves = {
            'tuck': {
                'left': [-1.0, -2.07, 3.0, 2.55, 0.0, 0.01, 0.0],
                'right': [1.0, -2.07, -3.0, 2.55, -0.0, 0.01, 0.0],
            },
            'untuck': {
                'left': [-0.08, -1.0, -1.19, 1.94, 0.67, 1.03, -0.50],
                'right': [0.08, -1.0, 1.19, 1.94, -0.67, 1.03, 0.50],
            },
        }

        collision_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._collide_lsub = node.create_subscription(
            CollisionAvoidanceState,
            'robot/limb/left/collision_avoidance_state',
            lambda msg: self._update_collision(msg, 'left'),
            qos_profile=collision_qos,
        )

        self._collide_rsub = node.create_subscription(
            CollisionAvoidanceState,
            'robot/limb/right/collision_avoidance_state',
            lambda msg: self._update_collision(msg, 'right'),
            qos_profile=collision_qos,
        )

        self._disable_pub = {
            'left': node.create_publisher(Empty, 'robot/limb/left/suppress_collision_avoidance', state_qos),
            'right': node.create_publisher(Empty, 'robot/limb/right/suppress_collision_avoidance', state_qos),
        }
        self._rs = baxter_interface.RobotEnable(node)
        self._enable_pub = node.create_publisher(Bool, 'robot/set_super_enable', state_qos)

    def _update_collision(self, data, limb):
        self._arm_state['collide'][limb] = len(data.collision_object) > 0
        self._check_arm_state()

    def _check_arm_state(self):
        """
        Check for goals and behind collision field.

        If s1 joint is over the peak, collision will need to be disabled
        to get the arm around the head-arm collision force-field.
        """

        def diff_check(a, b):
            return abs(a - b) <= self._tuck_threshold

        for limb in self._limbs:
            angles = [self._arms[limb].joint_angle(joint) for joint in self._arms[limb].joint_names()]

            # Check if in a goal position
            untuck_goal = map(diff_check, angles, self._joint_moves['untuck'][limb])
            tuck_goal = map(diff_check, angles[0:2], self._joint_moves['tuck'][limb][0:2])
            if all(untuck_goal):
                self._arm_state['tuck'][limb] = 'untuck'
            elif all(tuck_goal):
                self._arm_state['tuck'][limb] = 'tuck'
            else:
                self._arm_state['tuck'][limb] = 'none'

            # Check if shoulder is flipped over peak
            self._arm_state['flipped'][limb] = self._arms[limb].joint_angle(limb + '_s1') <= self._peak_angle

    def _prepare_to_tuck(self):
        # If arms are in "tucked" state, disable collision avoidance
        # before enabling robot, to avoid arm jerking from "force-field".
        head = baxter_interface.Head(self._node)
        start_disabled = not self._rs.state().enabled

        def at_goal():
            return abs(head.pan()) <= baxter_interface.settings.HEAD_PAN_ANGLE_TOLERANCE

        self._node.get_logger().info('Moving head to neutral position')
        while not at_goal() and rclpy.ok():
            if start_disabled:
                [pub.publish(Empty()) for pub in self._disable_pub.values()]
            if not self._rs.state().enabled:
                self._enable_pub.publish(Bool(data=True))
            head.set_pan(0.0, 0.5, timeout=0)
            self._tuck_rate.sleep()

        if start_disabled:
            while self._rs.state().enabled and rclpy.ok():
                [pub.publish(Empty()) for pub in self._disable_pub.values()]
                self._enable_pub.publish(Bool(data=False))
                self._tuck_rate.sleep()

    def _move_to(self, tuck, disabled):
        if any(disabled.values()):
            [pub.publish(Empty()) for pub in self._disable_pub.values()]
        while any(self._arm_state['tuck'][limb] != goal for limb, goal in tuck.items()) and rclpy.ok():
            if not self._rs.state().enabled:
                self._enable_pub.publish(True)
            for limb in self._limbs:
                if disabled[limb]:
                    self._disable_pub[limb].publish(Empty())
                if limb in tuck:
                    self._arms[limb].set_joint_positions(
                        dict(zip(self._arms[limb].joint_names(), self._joint_moves[tuck[limb]][limb]))
                    )
            self._check_arm_state()
            self._tuck_rate.sleep()

        if any(self._arm_state['collide'].values()):
            self._rs.disable()
        return

    def supervised_tuck(self):
        # Update our starting state to check if arms are tucked
        self._prepare_to_tuck()
        self._check_arm_state()
        # Tuck Arms
        if self._tuck:
            # If arms are already tucked, report this to user and exit.
            if all(self._arm_state['tuck'][limb] == 'tuck' for limb in self._limbs):
                self._node.get_logger().info("Tucking: Arms already in 'Tucked' position.")
                self._done = True
                return
            else:
                self._node.get_logger().info('Tucking: One or more arms not Tucked.')
                any_flipped = not all(self._arm_state['flipped'].values())
                if any_flipped:
                    self._node.get_logger().info(
                        'Moving to neutral start position with collision %s.', 'on' if any_flipped else 'off'
                    )
                # Move to neutral pose before tucking arms to avoid damage
                self._check_arm_state()
                actions = dict()
                disabled = {'left': True, 'right': True}
                for limb in self._limbs:
                    if not self._arm_state['flipped'][limb]:
                        actions[limb] = 'untuck'
                        disabled[limb] = False
                self._move_to(actions, disabled)

                # Disable collision and Tuck Arms
                self._node.get_logger().info('Tucking: Tucking with collision avoidance off.')
                actions = {'left': 'tuck', 'right': 'tuck'}
                disabled = {'left': True, 'right': True}
                self._move_to(actions, disabled)
                self._done = True
                return

        # Untuck Arms
        else:
            # If arms are tucked disable collision and untuck arms
            if any(self._arm_state['flipped'].values()):
                self._node.get_logger().info(
                    'Untucking: One or more arms Tucked; Disabling Collision Avoidance and untucking.'
                )
                self._check_arm_state()
                suppress = deepcopy(self._arm_state['flipped'])
                actions = {'left': 'untuck', 'right': 'untuck'}
                self._move_to(actions, suppress)
                self._done = True
                return
            # If arms already untucked, move to neutral location
            else:
                self._node.get_logger().info('Untucking: Arms already Untucked; Moving to neutral position.')
                self._check_arm_state()
                suppress = deepcopy(self._arm_state['flipped'])
                actions = {'left': 'untuck', 'right': 'untuck'}
                self._move_to(actions, suppress)
                self._done = True
                return

    def clean_shutdown(self):
        """Handles ROS shutdown (Ctrl-C) safely."""
        if not self._done:
            self._node.get_logger().warn('Aborting: Shutting down safely...')
        if any(self._arm_state['collide'].values()):
            while self._rs.state().enabled:
                [pub.publish(Empty()) for pub in self._disable_pub.values()]
                self._enable_pub.publish(False)
                self._tuck_rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    tuck_group = parser.add_mutually_exclusive_group(required=True)
    tuck_group.add_argument('-t', '--tuck', dest='tuck', action='store_true', default=False, help='tuck arms')
    tuck_group.add_argument('-u', '--untuck', dest='untuck', action='store_true', default=False, help='untuck arms')
    args = parser.parse_args()
    tuck = args.tuck
    rclpy.init()
    node = Node('rsdk_tuck_arms')
    node.get_logger().info('Initializing node... ')

    node.get_logger().info(f'{"T" if tuck else "Unt"}ucking arms')
    tucker = Tuck(node, tuck)
    rclpy.get_default_context().on_shutdown(tucker.clean_shutdown)

    tucker.supervised_tuck()
    node.get_logger().info('Finished tuck')


if __name__ == '__main__':
    main()
