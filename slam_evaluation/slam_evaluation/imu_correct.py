#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

class ImuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')
        
        self.target_pub_rate = 200.0
        self.last_msg = Imu()

        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            10)
            
        self.timebase = self.get_clock().now()
        
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        timer_period = 1.0 / self.target_pub_rate
        self.timer = self.create_timer(timer_period , self.timer_callback)
        
        self.last_nanosec = -1

    def imu_callback(self, msg):
        # Store the latest IMU message
        self.last_msg = msg
        
    def timer_callback(self):
    
    	current_time = self.get_clock().now()
    	timestamp = current_time - self.timebase 
    	     
    	if timestamp.nanoseconds < 0:
        	self.get_logger().warn("Negative timestamp encountered; resetting timebase.")
        	self.timebase = current_time
        	timestamp = current_time - self.timebase
        	
    	time_msg = Time()
    	time_msg.sec = int(timestamp.nanoseconds // 1e9)
    	time_msg.nanosec = int(timestamp.nanoseconds % 1e9)
    	
    	if time_msg.nanosec <= self.last_nanosec:
    		time_msg.nanosec = (self.last_nanosec + 5000000) % 1_000_000_000
    	
    	if time_msg.nanosec < 0:
        	self.get_logger().error("Negative nanosec calculated. Resetting to zero.")
        	time_msg.nanosec = 0
    	
    	self.last_nanosec = time_msg.nanosec
        	    	
    	header = Header()
    	header.stamp = time_msg
    	header.frame_id = "mara_imu_link" 
    	
    	imu_msg = Imu()
    	imu_msg.header = header
    	
    	imu_msg.orientation.x = self.last_msg.orientation.x
    	imu_msg.orientation.y = self.last_msg.orientation.y
    	imu_msg.orientation.z = self.last_msg.orientation.z
    	imu_msg.orientation.w = self.last_msg.orientation.w
    	
    	imu_msg.angular_velocity.x = self.last_msg.angular_velocity.x
    	imu_msg.angular_velocity.y = self.last_msg.angular_velocity.y
    	imu_msg.angular_velocity.z = self.last_msg.angular_velocity.z
    	
    	imu_msg.linear_acceleration.x = self.last_msg.linear_acceleration.x
    	imu_msg.linear_acceleration.y = self.last_msg.linear_acceleration.y
    	imu_msg.linear_acceleration.z = self.last_msg.linear_acceleration.z
    	
    	imu_msg.orientation_covariance = self.last_msg.orientation_covariance 
    	imu_msg.angular_velocity_covariance = self.last_msg.angular_velocity_covariance  
    	imu_msg.linear_acceleration_covariance = self.last_msg.linear_acceleration_covariance  
    	
    	self.publisher_.publish(imu_msg)
#        self.get_logger().info(f'Published IMU message at {header.stamp.sec}.{header.stamp.nanosec}')


def main(args=None):

    rclpy.init(args=args)
    node = ImuRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
