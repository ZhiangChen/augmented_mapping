import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from transitions import Machine
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped
from vision_msgs.msg import BoundingBox2D, Detection2DArray, Detection2D, Pose2D
from geometry_msgs.msg import PoseStamped, PoseArray, Pose


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
        states = ['idle', 'position', 'tracking', 'user_input', 'mapping']
        
        # Define the state machine
        self.machine = Machine(model=self, states=states, initial='idle')
        self.machine.add_transition('start_moving', 'idle', 'position', after='send_target_pose')
        self.machine.add_transition('position_reached', 'position', 'tracking', after='start_tracking')
        self.machine.add_transition('pbr_determination', 'tracking', 'user_input', after='detections_callback')
        self.machine.add_transition('start_mapping', 'user_input', 'mapping', after='transformed_bbox_callback')
        self.machine.add_transition('end_mapping', 'mapping', 'idle')
        
        # Position Control
        ## Publish the targer position 
        self.pose_publisher = self.create_publisher(PoseStamped, '/target_pos', qos_profile)
        ## Receive service when position control is finished
        self.done_service = self.create_service(Trigger, 'position_control/done', self.done_callback)

        # PBR Detection
        ## Start yolov7 detections
        self.yolov7_ros_client = self.create_client(Trigger, 'yolov7_ros/start')
        ## Gets detections message from tracker
        self.detections_subscriber = self.create_subscriber(Detection2DArray, 
                                    "/iou_tracker/bounding_boxes", self.detections_callback, 10)
        self.detected_pbrs = None

        # Obtains user input for PBR number
        self.pbr_subscriber = self.create_subscriber(Int32, '/pbr_number', self.pbr_number_callback, qos_profile)
        self.bbox_publisher = self.create_publisher(Detection2D, '/pbr_bbox_original')
        self.bbox_transformed_subscriber = self.create_subscriber(
                                            PoseArray, '/target_bbox', self.transformed_bbox_callback, 10)
        
        # PBR Mapping
        self.bbox_velocity_publisher = self.create_publisher(
                                        PoseArray, '/target_bbox', qos_profile)
        
        self.get_logger().info("State Machine Initialized in 'idle' state.")
        self.send_target_pose()

    def send_target_pose(self):
        target_pose = PoseStamped()
        target_x = 5.0
        target_y = 5.0
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        self.pose_publisher.publish(target_pose)
        self.get_logger().info("Sent target position to position control.")

    def done_callback(self, request, response):
        self.position_reached()
        self.get_logger().info("Position reached. Transitioning to tracking.")
        response.success = True
        response.message = "Position reached. Transitioning to tracking."
        return response

    def start_tracking(self):
        if not self.yolov7_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("YOLOv7 detection service not available.")
            return

        request = Trigger.Request()
        future = self.yolov7_client.call_async(request)
        future.add_done_callback(self.tracking_started_callback)


    def tracking_started_callback(self, future):
        if future.result().success:
            self.get_logger().info("Started object tracking.")
        else:
            self.get_logger().error("Failed to start object tracking.")

    def detections_callback(self, detected_pbrs):
        self.detected_pbrs = detected_pbrs
        for pbr in detected_pbrs.detections:
            pbr_id = pbr.results.id
            pbr_score = pbr.results.score
            x,y = pbr.bbox.center.x, pbr.bbox.center.y
            self.get_logger().info("PBR " + str(pbr_id) + " detected at " + str(x) + ", " + str(y) + " with score " + str(pbr_score))


    def pbr_number_callback(self, pbr_num):
        for pbr in self.detected_pbrs:
            if (pbr.results.id == pbr_num):
                self.bbox_publisher.publish(pbr)
    
    def transformed_bbox_callback(self, bbox_msg):
        self.bbox_velocity_publisher.publish(bbox_msg)



def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
