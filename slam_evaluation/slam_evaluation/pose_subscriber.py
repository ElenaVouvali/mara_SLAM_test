# Copyright 2016 Open Source Robotics Foundation, Inc.
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


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np


class PoseSubscriber(Node):
    def __init__(self):
    
        super().__init__('pose_subscriber')
        
        # Subscription to the /ground_truth_pose topic
        self.ground_truth_subscription = self.create_subscription(
            PoseStamped,
            '/ground_truth_pose',
            self.ground_truth_callback,
            10)
        
        # Subscription to the /lidar_odometry topic
        self.lidar_odometry_subscription = self.create_subscription(
            Odometry, 
            '/lidar_odometry',
            self.lidar_odometry_callback,
            10)
            
        self.ground_truth_pose = None
        self.lidar_odometry_pose = None
        
        self.get_logger().info('PoseSubscriber node has been started and is subscribing to /ground_truth_pose and /lidar_odometry.')


    def ground_truth_callback(self, msg):
    	
    	self.ground_truth_pose = msg.pose 
    	self.calculate_errors()   
	
#        self.get_logger().info(f'Received Ground Truth Pose:\n'
#                               f'Position: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}\n'
#                               f'Orientation: x={msg.pose.orientation.x}, y={msg.pose.orientation.y}, '
#                               f'z={msg.pose.orientation.z}, w={msg.pose.orientation.w}')

    def lidar_odometry_callback(self, msg):
    
    	self.lidar_odometry_pose = msg.pose.pose
    	self.calculate_errors()

#        self.get_logger().info(f'Received Lidar Odometry Pose:\n'
#                               f'Position: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z}\n'
#                               f'Orientation: x={msg.pose.pose.orientation.x}, y={msg.pose.pose.orientation.y}, '
#                               f'z={msg.pose.pose.orientation.z}, w={msg.pose.pose.orientation.w}')
                               
        
    
    def calculate_errors(self):
    
        if self.ground_truth_pose is not None and self.lidar_odometry_pose is not None:

            ape_error = np.linalg.norm(np.array([
                self.ground_truth_pose.position.x - self.lidar_odometry_pose.position.x,
                self.ground_truth_pose.position.y - self.lidar_odometry_pose.position.y,
                self.ground_truth_pose.position.z - self.lidar_odometry_pose.position.z
            ]))


            self.get_logger().info(f'APE: {ape_error}')
        
            


def main(args=None):

    try:
        rclpy.init(args=args)
        pose_subscriber_node = PoseSubscriber()
        rclpy.spin(pose_subscriber_node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)

    pose_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

