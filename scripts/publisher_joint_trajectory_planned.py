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
from std_msgs.msg import Bool



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
            self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
            self.create_subscription(JointTrajectory, "/planned_trajectory", self.joint_path_callback, 10)
            self.create_subscription(Bool, "/emergency_stop", self.emergency_stop_callback, 10)

        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False
        self.trajectory_received = False
        self.planned_trajectory = None
        self.stop_trajectory = False 
        self.execution_complete = True
        self.current_joint_state = None
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.execution_status_pub = self.create_publisher(Bool, "/execution_status", 10)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)

    def emergency_stop_callback(self, msg):
        """Función que activa la parada de emergencia"""
        self.emergency_stop = msg
        self.stop_trajectory = True
        self.execution_complete = False
        self.get_logger().warn("Emergency stop triggered! Stopping the robot.")
        # Aquí puedes mandar comandos para detener la trayectoria
        self.stop_joint_trajectory()        

    def stop_joint_trajectory(self):
        """Comando para detener el robot"""
        # Publica un mensaje vacío o comando de parada al controlador de la trayectoria
        stop_msg = JointTrajectory()
        stop_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        stop_msg.points = []  # Vacío o "detener movimiento"
        self.publisher_.publish(stop_msg)
        self.get_logger().info("Sent stop command to the robot.")
        self.execution_complete = True

    def timer_callback(self):
        self.monitor_execution()

        if self.stop_trajectory:
            self.get_logger().warn("Trajectory stop triggered, halting movement.")
            return
        
        if not self.starting_point_ok:
            self.get_logger().warn("Start configuration is not within configured limits!")
            return
        
        execution_msg = Bool()
        execution_msg.data = self.execution_complete
        self.execution_status_pub.publish(execution_msg)       

        if self.trajectory_received and self.planned_trajectory:
            self.get_logger().info("Publishing received trajectory to controller...")
            self.publisher_.publish(self.planned_trajectory)
            self.trajectory_received = False  # evitar repetir envío
            self.execution_complete = False  # Marca la flag como falsa cuando una nueva trayectoria empieza
        else:
            self.get_logger().info("No planned trajectory received yet.")

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

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
        # self.execution_complete = False
        self.get_logger().info("Planned trajectory received and stored.")
        return
    
    def monitor_execution(self):
        """Verifica si el robot ha alcanzado la posición final"""
        # Simula una verificación de que el robot ha alcanzado su meta.
        # Este es un lugar donde puedes agregar un mecanismo real como un ActionServer
        # o simplemente verificar la posición en el JointState, por ejemplo:

        self.get_logger().info("Monitoring execution.")
        
        if self.current_joint_state:  # Solo lo haces si el JointState está disponible
            self.get_logger().info("Current joint state received.")
            # self.get_logger().info(f"Current joint state: {self.current_joint_state}")
            goal_reached = True
            if self.planned_trajectory is not None:
                # self.get_logger().info(f"Planned trajectory: {self.planned_trajectory}")
                # self.get_logger().info(f"Current joint state position 5: {self.current_joint_state.position[3:5]}")
                current_joint_values = [self.current_joint_state.position[-1], self.current_joint_state.position[0], self.current_joint_state.position[1], self.current_joint_state.position[2], self.current_joint_state.position[3], self.current_joint_state.position[4]]
                # self.get_logger().info(f"Current joint values: {current_joint_values}")
                for idx, goal_position in enumerate(self.planned_trajectory.points[-1].positions):
                    # self.get_logger().info(f"Goal positons: {goal_position}")
                    if abs(current_joint_values[idx] - goal_position) > 0.01:  # Tolerancia
                        goal_reached = False
                        break
            
            if goal_reached:
                self.execution_complete = True
                self.get_logger().info("Goal reached! Execution complete.")
            else:
                self.get_logger().info("Robot has not yet reached goal. Continuing monitoring.")

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
















# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from control_msgs.action import FollowJointTrajectory
# from rclpy.action import ActionClient
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from std_msgs.msg import Bool
# from geometry_msgs.msg import Pose
# import time

# class ActionClientNode(Node):
#     def __init__(self):
#         super().__init__('publisher_joint_trajectory_planned')

#         # Crear Action Client
#         self._action_client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')

#         # Crear el publisher para la ejecución de la trayectoria
#         self.execution_status_pub = self.create_publisher(Bool, '/execution_status', 10)

#         self.get_logger().info("Action client node initialized.")

#     def goal_callback(self, msg: Pose):
#         """Callback para recibir goals desde el topic y enviarlos al Action Server"""
#         self.send_goal(msg)

#     def send_goal(self, goal_pose: Pose):
#         """Enviar una meta al Action Server"""
#         goal_msg = FollowJointTrajectory.Goal()

#         joint_trajectory = JointTrajectory()
#         joint_trajectory.joint_names = [
#             'shoulder_pan_joint',
#             'shoulder_lift_joint',
#             'elbow_joint',
#             'wrist_1_joint',
#             'wrist_2_joint',
#             'wrist_3_joint'
#         ]

#         target_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
#         target_orientation = [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w]

#         point = JointTrajectoryPoint()
#         point.positions = target_position + target_orientation
#         point.time_from_start = rclpy.duration.Duration(seconds=5).to_msg()
#         joint_trajectory.points.append(point)

#         goal_msg.trajectory = joint_trajectory

#         self._action_client.wait_for_server()
#         self.get_logger().info('Sending goal...')
#         self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f"Received feedback: {feedback_msg.feedback}")

#     def result_callback(self, result):
#         self.get_logger().info(f"Execution result: {result}")

#     def execute(self, goal_pose: Pose):
#         self.send_goal(goal_pose)

#         # Publicar el estado de la ejecución
#         execution_msg = Bool()
#         execution_msg.data = True  # Se puede ajustar si se desea mantener el estado
#         self.execution_status_pub.publish(execution_msg)

# def main(args=None):
#     rclpy.init(args=args)

#     action_client_node = ActionClientNode()

#     goal_pose = Pose()
#     goal_pose.position.x = 1.0
#     goal_pose.position.y = 0.5
#     goal_pose.position.z = 0.0
#     goal_pose.orientation.x = 0.0
#     goal_pose.orientation.y = 0.0
#     goal_pose.orientation.z = 0.0
#     goal_pose.orientation.w = 1.0

#     action_client_node.execute(goal_pose)

#     rclpy.spin(action_client_node)

# if __name__ == '__main__':
#     main()




