import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from transitions import Machine
from std_msgs.msg import Int32, Empty
from std_srvs.srv import Trigger
# from state_machine.srv import BoundingBox
from geometry_msgs.msg import Point, PoseStamped
from vision_msgs.msg import BoundingBox2D, Detection2DArray, Detection2D, Pose2D
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from custom_msgs.srv import SetBoundingBox
import time






class StateMachineNode(Node):
   def __init__(self):
       super().__init__('state_machine')


       qos_profile = QoSProfile(
           reliability=ReliabilityPolicy.BEST_EFFORT,
           durability=DurabilityPolicy.TRANSIENT_LOCAL,
           history=HistoryPolicy.KEEP_LAST,
           depth=1
       )


       # State machine states
       # idle: the state machine initially starts out in idle state
       # position: the state machine transitions to position mode when the lift height is reached
       # tracking: the state machine transitions to tracking mode when the position is reached
       # user_input: the state machine is waiting for user input
       # mapping: the state machine is in mapping mode
       states = ['idle', 'position', 'tracking', 'user_input', 'bbox_sent','mapping']
      
       # Define the state machine
       self.machine = Machine(model=self, states=states, initial='idle')
       self.machine.add_transition('height_reached_callback', 'idle', 'position', after='send_target_pose')
       self.machine.add_transition('position_reached_callback', 'position', 'tracking', after='start_tracking')
       self.machine.add_transition('detections_received', 'tracking', 'user_input')
       self.machine.add_transition('transformed_bbox_received', 'user_input', 'bbox_sent', after='velocity_control_trigger')
       self.machine.add_transition('go_to_mapping', 'bbox_sent', 'mapping')
       self.machine.add_transition('end_mapping', 'mapping', 'idle')
      
       # Position Control
       ## Publish the targer position
       self.pose_publisher = self.create_publisher(PoseStamped, '/target_pos', qos_profile)
       ## Receive service when position control is finished
       self.done_service = self.create_service(Trigger, 'position_control/done', self.position_reached_callback)
       self.ready_service = self.create_service(Trigger, 'position_control/ready', self.height_reached_callback)




       # PBR Detection
       ## Start yolov7 detections
       self.yolov7_ros_publisher = self.create_publisher(Point, 'state_machine/tracking_start', 10)
       ## Gets detections message from tracker
       self.detections_subscriber = self.create_subscription(Detection2DArray,
                                   "/iou_tracker/bounding_boxes", self.detections_callback, 10)
       self.detected_pbrs = None




       # Obtains user input for PBR number
       #self.pbr_subscriber = self.create_subscription(Int32, '/pbr_number', self.pbr_number_callback, qos_profile)
       self.bbox_publisher = self.create_publisher(Detection2D, '/pbr_bbox_original', 10)
       self.bbox_transformed_subscriber = self.create_subscription(
                                           PoseArray, '/target_bbox', self.transformed_bbox_callback, 10)
      
       # PBR Mapping
       self.bbox_fuel_publisher = self.create_publisher(PoseArray, '/exploration_node/pose_array_topic', 10)
       self.velocity_control_pub = self.create_publisher(Empty, 'state_machine/velocity_control', 10)
      
       self.get_logger().info("State Machine Initialized in 'idle' state.")




   def send_target_pose(self):
       target_pose = PoseStamped()
       target_x = 10.0
       target_y = 3.0
       target_pose.pose.position.x = target_x
       target_pose.pose.position.y = target_y
       self.pose_publisher.publish(target_pose)
       self.get_logger().info("Sent target position to position control.")


   def height_reached_callback(self, request, response):
       self.get_logger().info("Lift height reached. Transitioning to position.")


       self.trigger('height_reached_callback')


       response.success = True
       response.message = "Position reached. Transitioning to position."
       return response




   def position_reached_callback(self, request, response):
       self.get_logger().info("Position reached. Transitioning to tracking.")
       response.success = True
       self.trigger('position_reached_callback')
       response.message = "Position reached. Transitioning to tracking."
       return response




   def start_tracking(self):
       self.get_logger().info("Starting object tracking.")


       new_message = Point()
       new_message.x = 0.0
       new_message.y = 0.0
       new_message.z = 0.0
       self.yolov7_ros_publisher.publish(new_message)
       self.get_logger().info(f"Currenct state at start_tracking: {self.state}")






   def tracking_started_callback(self, future):
       if future.result().success:
           self.get_logger().info("Started object tracking.")
       else:
           self.get_logger().error("Failed to start object tracking.")




   def detections_callback(self, detected_pbrs):
       if self.state == "tracking":
           self.get_logger().info(f"Currenct state before triggering: {self.state}")
           self.detected_pbrs = detected_pbrs
           for pbr in detected_pbrs.detections:
               self.get_logger().info(str(pbr))
               pbr_id = pbr.results[0].hypothesis.class_id
               pbr_score = pbr.results[0].hypothesis.score
               x,y = pbr.bbox.center.position.x, pbr.bbox.center.position.y
               self.get_logger().info("PBR " + str(pbr_id) + " detected at " + str(x) + ", " + str(y) + " with score " + str(pbr_score))
           self.bbox_publisher.publish(detected_pbrs.detections[0])
           self.trigger('detections_received')
           self.get_logger().info("Received detections from tracker.")




   # def pbr_number_callback(self, pbr_num):
   #     for pbr in self.detected_pbrs:
   #         if (pbr.results.id == pbr_num):
   #             self.bbox_publisher.publish(pbr)
  


   def transformed_bbox_callback(self, bbox_msg):
       self.get_logger().info("Received transformed bounding box message.")
       if self.state == 'user_input':
           bbox_copy = PoseArray()
           bbox_copy.header = bbox_msg.header


           for pbr in bbox_msg.poses:
               bbox_copy.poses.append(pbr)
              
           self.bbox_fuel_publisher.publish(bbox_copy)
           self.get_logger().info("Bounding box " + str(bbox_copy.poses))
           self.get_logger().info(f"[DEBUG] State before trigger: {self.state}")
           time.sleep(10)
           result = self.trigger('transformed_bbox_received')
           self.get_logger().info(f"[DEBUG] Trigger result: {self.state}")


          


   def velocity_control_trigger(self):
       self.get_logger().info("Velocity control triggered.")
       empty_msg = Empty()
       self.velocity_control_pub.publish(empty_msg)
       self.trigger('go_to_mapping')


def main(args=None):
   rclpy.init(args=args)
   node = StateMachineNode()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()






if __name__ == '__main__':
   main()





