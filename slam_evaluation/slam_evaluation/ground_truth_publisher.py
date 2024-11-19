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
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetEntityState
from std_msgs.msg import Header



class GroundTruth(Node):

    def __init__(self):    
    
    	super().__init__('ground_truth')
    	self.service_client = self.create_client(srv_type = GetEntityState, srv_name = '/gazebo/get_entity_state')
    	
    	while not self.service_client.wait_for_service(timeout_sec=1.0): 
    		self.get_logger().info(f'service {self.service_client.srv_name} not available, waiting...')
    	
    	self.publisher_ = self.create_publisher(PoseStamped, 'ground_truth_pose', 10)
    	
    	self.future: Future = None    	
    	timer_period: float = 0.15    	
    	self.timer = self.create_timer(timer_period_sec=timer_period, callback=self.timer_callback)


    def timer_callback(self):
    	
    	"""Method that is periodically called by the timer."""
    	
    	request = GetEntityState.Request()
    	request.name = 'mara_robot'
    	request.reference_frame = 'world'
    	
    	if self.future is not None and not self.future.done():
    		self.future.cancel() 
    		self.get_logger().info("Service Future canceled. The Node took too long to handle the service call."
    					"Is the Service Server still alive?")
    	
    	self.future = self.service_client.call_async(request)
    	self.future.add_done_callback(self.handle_service_response)


    def handle_service_response(self, future: Future):

        """Callback for the future, that will be called when it is done"""

        response = future.result()
        
        if response is not None and response.success:
                pose = response.state.pose
                timestamp = response.header.stamp
                
                pose_stamped = PoseStamped()
                pose_stamped.header = Header()
                pose_stamped.header.stamp = timestamp
                pose_stamped.header.frame_id = 'world'
                pose_stamped.pose = pose
                
                self.publisher_.publish(pose_stamped)
                #self.get_logger().info(f'Published PoseStamped:\n{pose_stamped}')       
                
        else:
        	self.get_logger().info(f'Failed to get entity state')
        	


def main(args=None):

    try:
        rclpy.init(args=args)
        ground_truth_node = GroundTruth()
        rclpy.spin(ground_truth_node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)

    ground_truth_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
