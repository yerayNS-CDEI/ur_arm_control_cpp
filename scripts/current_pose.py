import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

class UR10ePose(Node):
    def __init__(self):
        super().__init__('ur10e_pose_node')
        self.move_group = MoveGroupCommander("manipulator")  # Use your MoveIt group name

    def get_end_effector_pose(self):
        pose = self.move_group.get_current_pose()
        self.get_logger().info(f"End Effector Pose: {pose}")
        return pose

def main():
    rclpy.init()
    node = UR10ePose()
    node.get_end_effector_pose()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
