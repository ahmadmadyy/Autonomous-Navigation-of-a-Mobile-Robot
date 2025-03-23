import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import os
import sys

class MoveToSpot(Node):
    def __init__(self):
        super().__init__('move_to_spot')

        # Get the spot name from ROS2 parameters
        self.declare_parameter('spot_name', '')
        self.spot_name = self.get_parameter('spot_name').get_parameter_value().string_value

        # Load the spot list from YAML
        self.spot_list = self.load_spots()

        if self.spot_name not in self.spot_list:
            self.get_logger().error(f"Spot '{self.spot_name}' not found in the spot list!")
            sys.exit(1)

        # Create an action client for NavigateToPose
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Wait for the action server to be available
        self.get_logger().info("Waiting for Nav2 NavigateToPose action server...")
        self.client.wait_for_server()

        # Get the target pose
        target_pose = self.get_target_pose()

        # Send the goal
        self.send_goal(target_pose)

    def load_spots(self):
        """Load spots from spots.yaml."""
        spots_file = os.path.expanduser("~/ros2_ws/src/project_path_planning/config/spots.yaml")
        with open(spots_file, 'r') as file:
            spots_data = yaml.safe_load(file)
        return spots_data.get('move_to_spot', {}).get('ros__parameters', {})

    def get_target_pose(self):
        """Convert spot data to a PoseStamped message."""
        spot_data = self.spot_list[self.spot_name]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = spot_data['position']['x']
        pose.pose.position.y = spot_data['position']['y']
        pose.pose.position.z = spot_data['position']['z']

        pose.pose.orientation.x = spot_data['orientation']['x']
        pose.pose.orientation.y = spot_data['orientation']['y']
        pose.pose.orientation.z = spot_data['orientation']['z']
        pose.pose.orientation.w = spot_data['orientation']['w']

        return pose

    def send_goal(self, target_pose):
        """Send the navigation goal to Nav2."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.get_logger().info(f"Sending robot to: {self.spot_name}")

        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback when goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted! Navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback when the robot reaches the goal."""
        result = future.result()
        if result:
            self.get_logger().info("Navigation successful!")
        else:
            self.get_logger().error("Navigation failed!")

def main():
    rclpy.init()
    node = MoveToSpot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
