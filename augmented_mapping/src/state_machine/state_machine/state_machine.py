import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from transitions import Machine
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped


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
        states = ['idle', 'moving', 'tracking']
        
        # Define the state machine
        self.machine = Machine(model=self, states=states, initial='idle')
        self.machine.add_transition('start_moving', 'idle', 'moving', after='send_target_pose')
        self.machine.add_transition('position_reached', 'moving', 'tracking', after='start_tracking')
        self.machine.add_transition('stop_tracking', 'tracking', 'idle')
        
        # Service clients
        self.pose_publisher = self.create_publisher(PoseStamped, '/target_pos', qos_profile)
        self.yolov7_ros_client = self.create_client(Trigger, 'yolov7_ros/start')
        
        # Service server to receive position control completion signal
        self.done_service = self.create_service(Trigger, 'position_control/done', self.done_callback)
        
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

        request = StartDetection.Request()
        future = self.yolov7_client.call_async(request)
        future.add_done_callback(self.tracking_started_callback)


    def tracking_started_callback(self, future):
        if future.result().success:
            self.get_logger().info("Started object tracking.")
        else:
            self.get_logger().error("Failed to start object tracking.")





def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
