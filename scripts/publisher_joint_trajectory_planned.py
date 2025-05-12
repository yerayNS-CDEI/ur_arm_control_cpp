#!/usr/bin/env python3
# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Denis Štogl, Lovro Ivanov
#

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherJointTrajectory(Node):
    ## Subscripcion a un topic que de la posicion actual del robot


    def __init__(self):
        super().__init__("publisher_joint_trajectory_planned")
        # Declare all parameters
        self.declare_parameter("controller_name", "position_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 1)
        self.declare_parameter("joints", [""])
        self.declare_parameter("check_starting_point", True)

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
            self.create_subscription(
                JointTrajectory, "/planned_trajectory", self.joint_path_callback, 10
            )

        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False
        self.trajectory_received = False
        self.planned_trajectory = None        

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)

    def timer_callback(self):

        if not self.starting_point_ok:
            self.get_logger().warn("Start configuration is not within configured limits!")
            return
        
        if self.trajectory_received and self.planned_trajectory:
            self.get_logger().info("Publishing received trajectory to controller...")
            self.publisher_.publish(self.planned_trajectory)
            self.trajectory_received = False  # evitar repetir envío
        else:
            self.get_logger().info("No planned trajectory received yet.")

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:   # Si se elimina esta linea, se evalua en cada punto

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    # self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    self.get_logger().warn(f"Value stp0: {self.starting_point[enum][0]}, Value stp0: {self.starting_point[enum][1]}, Value pos: {msg.position[idx]}")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return
        
    def joint_path_callback(self, msg):
        if not self.starting_point_ok:
            self.get_logger().warn("Received trajectory, but starting point is not valid!")
            return

        self.planned_trajectory = msg
        self.trajectory_received = True
        self.get_logger().info("Planned trajectory received and stored.")
        return

def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    try:
        rclpy.spin(publisher_joint_trajectory)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Keyboard interrupt received. Shutting down node.")
    except Exception as e:
        print(f"Unhandled exception: {e}")


if __name__ == "__main__":
    main()
