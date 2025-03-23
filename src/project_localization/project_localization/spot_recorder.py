from custom_interfaces.srv import SaveSpot
from geometry_msgs.msg import PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory
import os

# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node
import pandas as pd

class SaveSpotServer(Node):

    def __init__(self):
        super().__init__('spot_recorder')
        self.save_spot_srv = self.create_service(SaveSpot, 'record_spot', self.save_spot_callback)
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_subscriber_callback, 10)

        self.spots_register = {'label':[], 'coordinate_x':[], 'coordinate_y':[], 'orientation_z':[], 'orientation_w':[]}
        self.n_spots = 0
        self.service_finished = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_w = 0.0
        self.csv_dir = os.path.join(get_package_share_directory('project_mapping'), 'csv')
        self.spots_file_path = os.path.join(self.csv_dir, 'spots_file.csv')
    
    def save_spot_callback(self, request, response):
        if request.label == "end":
            spots_register_df = pd.DataFrame.from_dict(self.spots_register)
            spots_register_df.to_csv("spots_file.csv", index=False)
            response.navigation_successfull = True
            response.message = "Spots file has been saved with " + str(self.n_spots) + " spots registered"
            self.service_finished = True
        else:
            # response state
            self.spots_register['label'].append(request.label)
            self.spots_register['coordinate_x'].append(self.current_x)
            self.spots_register['coordinate_y'].append(self.current_y)
            self.spots_register['orientation_z'].append(self.current_z)
            self.spots_register['orientation_w'].append(self.current_w)
            response.navigation_successfull = True
            response.message = "Spot " + request.label + " has been registered"
            self.n_spots += 1
        
        # return the response parameter
        return response
    
    def pose_subscriber_callback(self, msg):
        # self.get_logger().info('inside callback')
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.orientation.z
        self.current_w = msg.pose.pose.orientation.w


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    service = SaveSpotServer()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    while not service.service_finished:
        rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()