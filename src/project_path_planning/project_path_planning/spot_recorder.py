import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import os

class SpotRecorder(Node):
    def __init__(self):
        super().__init__('spot_recorder')

        # Subscribe to the /initialpose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_callback,
            10
        )

        # Spots storage
        self.spots = {}

        # Set up YAML file path
        package_share_dir = os.path.expanduser("~/ros2_ws/src/project_path_planning/config")
        self.yaml_file_path = os.path.join(package_share_dir, 'spots.yaml')

        self.get_logger().info("Spot Recorder is running. Use 2D Pose Estimate in RViz to mark locations.")

    def pose_callback(self, msg):
        # Ask user to enter a label for the recorded spot
        label = input("Enter spot label (corner1, corner2, pedestrian): ").strip()

        if label not in ["corner1", "corner2", "pedestrian"]:
            self.get_logger().warn(f"Invalid label: {label}. Allowed: corner1, corner2, pedestrian")
            return

        # Extract position and orientation
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Store in dictionary
        self.spots[label] = {
            "position": {
                "x": position.x,
                "y": position.y,
                "z": position.z
            },
            "orientation": {
                "x": orientation.x,
                "y": orientation.y,
                "z": orientation.z,
                "w": orientation.w
            }
        }

        # Save to YAML file
        self.save_to_yaml()

        self.get_logger().info(f"Recorded spot: {label}")

    def save_to_yaml(self):
        with open(self.yaml_file_path, 'w') as file:
            yaml.dump(self.spots, file, default_flow_style=False)

def main():
    rclpy.init()
    node = SpotRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
