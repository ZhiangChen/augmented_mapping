import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from state_machine.srv import NotifyPositionReached, StartDetection
from position_control.srv import SetTargetPosition


class UAVStateMachine(Node):
    def __init__(self):
        super().__init__('uav_state_machine')


        # Define states
        self.IDLE = 'IDLE'
        self.MOVING = 'MOVING'
        self.WAITING = 'WAITING'
        self.DETECTING = 'DETECTING'
        self.COMPLETED = 'COMPLETED'


        # Initialize the current state
        self.current_state = self.IDLE


        # Create service clients
        self.position_client = self.create_client(SetTargetPosition, 'set_position')
        self.detection_client = self.create_client(StartDetection, 'start_detection')


        # Create publishers or subscribers if needed
        self.publisher = self.create_publisher(String, 'status', 10)


        # Create a timer to trigger state transitions
        self.timer = self.create_timer(2.0, self.timer_callback)


    def timer_callback(self):
        """Timer callback to handle state transitions based on current state."""
        if self.current_state == self.IDLE:
            self.get_logger().info('State: IDLE, sending position...')
            self.send_position_service()
            self.current_state = self.MOVING  # Transition to MOVING


        elif self.current_state == self.MOVING:
            self.get_logger().info('State: MOVING, waiting for position reach...')
            self.reach_position()  # Check if position is reached and transition to WAITING


        elif self.current_state == self.WAITING:
            self.get_logger().info('State: WAITING, starting detection...')
            self.start_detection_service()
            self.current_state = self.DETECTING  # Transition to DETECTING


        elif self.current_state == self.DETECTING:
            self.get_logger().info('State: DETECTING, process complete.')
            self.current_state = self.COMPLETED  # Transition to COMPLETED


        elif self.current_state == self.COMPLETED:
            self.get_logger().info('State: COMPLETED.')


    def send_position_service(self):
        """Send a service request to the position control to set the UAV's position."""
        if not self.position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Position service not available.')
            return


        request = SetPosition.Request()
        request.x = 10.0  # Example position
        request.y = 20.0
        self.position_client.call_async(request)


    def reach_position(self):
        """Check if the UAV has reached the target position."""
        # This could be extended to check the current position and compare it to the target
        # If the position is reached, trigger the transition to WAITING
        self.get_logger().info('Position reached, transitioning to WAITING.')


    def start_detection_service(self):
        """Send a service request to start the detection process."""
        if not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Detection service not available.')
            return


        request = StartDetection.Request()
        self.detection_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    state_machine = UAVStateMachine()
    rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
