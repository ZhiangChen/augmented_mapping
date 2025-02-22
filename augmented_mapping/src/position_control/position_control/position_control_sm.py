from px4_msgs.msg import TrajectorySetpoint
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint, TrajectorySetpoint
from geometry_msgs.msg import Quaternion, PoseStamped, Pose
from std_srvs.srv import Trigger
# from position_control.srv import SetTargetPosition
# from state_machine.srv import NotifyPositionReached  # Import service to notify state machine


class UAVPositionController(Node):
   def __init__(self):
       super().__init__('uav_position_controller')


       qos_profile = QoSProfile(
           reliability=ReliabilityPolicy.BEST_EFFORT,
           durability=DurabilityPolicy.TRANSIENT_LOCAL,
           history=HistoryPolicy.KEEP_LAST,
           depth=1
       )




       # Parameters
       self.lift_height = self.declare_parameter("lift_height", -5.0).value
       self.target_x = None
       self.target_y = None
       self.target_z = None
       self.kp = self.declare_parameter("kp", 1.0).value
       self.tolerance = self.declare_parameter("tolerance", 0.1).value
       self.offboard_setpoint_counter = 0




       # Current and target positions
       self.vehicle_local_position = None
       self.current_stage = "WAIT"
       self.vehicle_status = None


       # Service for setting target position
       self.target_position_subscriber = self.create_subscription(
           PoseStamped, '/target_pos', self.target_position_callback, qos_profile)




       # Client to notify state machine
       self.notify_state_machine_client = self.create_client(
           Trigger, 'position_control/done')




       # Publisher for TrajectorySetpoint
       self.offboard_control_mode_publisher = self.create_publisher(
           OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
       self.trajectory_setpoint_publisher = self.create_publisher(
           TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
       self.vehicle_command_publisher = self.create_publisher(
           VehicleCommand, '/fmu/in/vehicle_command', qos_profile)




       # Subscribers for current position
       # self.target_position_subscriber = self.create_subscription(PoseStamped, '/target_pos', self.target_position_callback, 10)
       self.vehicle_local_position_subscriber = self.create_subscription(
           VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
       self.vehicle_status_subscriber = self.create_subscription(
           VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)




       # Timer for the control loop
       self.timer = self.create_timer(0.1, self.control_loop)


   def vehicle_local_position_callback(self, vehicle_local_position):
       """Callback function for vehicle_local_position topic subscriber."""
       self.vehicle_local_position = vehicle_local_position


   def vehicle_status_callback(self, vehicle_status):
       """Callback function for vehicle_status topic subscriber."""
       self.vehicle_status = vehicle_status


   def arm(self):
       """Send an arm command to the vehicle."""
       self.publish_vehicle_command(
           VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
       self.get_logger().info('Arm command sent')


   def disarm(self):
       """Send a disarm command to the vehicle."""
       self.publish_vehicle_command(
           VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
       self.get_logger().info('Disarm command sent')


   def engage_offboard_mode(self):
       """Switch to offboard mode."""
       self.publish_vehicle_command(
           VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
       self.get_logger().info("Switching to offboard mode")


   def publish_offboard_control_heartbeat_signal(self):
       """Publish the offboard control mode."""
       msg = OffboardControlMode()
       msg.position = True
       msg.velocity = False
       msg.acceleration = False
       msg.attitude = False
       msg.body_rate = False
       msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
       self.offboard_control_mode_publisher.publish(msg)


   def publish_vehicle_command(self, command, **params) -> None:
       """Publish a vehicle command."""
       msg = VehicleCommand()
       msg.command = command
       msg.param1 = params.get("param1", 0.0)
       msg.param2 = params.get("param2", 0.0)
       msg.param3 = params.get("param3", 0.0)
       msg.param4 = params.get("param4", 0.0)
       msg.param5 = params.get("param5", 0.0)
       msg.param6 = params.get("param6", 0.0)
       msg.param7 = params.get("param7", 0.0)
       msg.target_system = 1
       msg.target_component = 1
       msg.source_system = 1
       msg.source_component = 1
       msg.from_external = True
       msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
       self.vehicle_command_publisher.publish(msg)




   def target_position_callback(self, msg):
       self.get_logger().info("target set")
       self.target_x = msg.pose.position.x
       self.target_y = msg.pose.position.y
       self.target_z = msg.pose.position.z
       self.current_stage = "LIFT"






   def set_target_position_callback(self, request, response):
       """Handles service request to move UAV."""
       self.target_x = request.x
       self.target_y = request.y
       self.target_z = request.z
       self.get_logger().info(f"Received target position: ({self.target_x}, {self.target_y}, {self.target_z})")


       self.offboard_setpoint_counter = 0
       self.current_stage = "LIFT"
       response.success = True
       return response


   def notify_state_machine(self):
       """Sends a service request to notify state machine when position is reached."""
       request = Trigger.Request()
       future = self.notify_state_machine_client.call_async(request)
       rclpy.spin_until_future_complete(self, future)
       if future.result() and future.result().success:
           self.get_logger().info("State machine notified successfully.")




   def control_loop(self):


       if self.offboard_setpoint_counter == 0:
           self.engage_offboard_mode()
           self.arm()
       self.offboard_setpoint_counter += 1


       self.publish_offboard_control_heartbeat_signal()


       setpoint_msg = TrajectorySetpoint()
       setpoint_msg.yaw = 1.57079




       if self.current_stage == "LIFT":
           if self.vehicle_local_position is not None:
               error_z = self.lift_height - self.vehicle_local_position.z




               if abs(error_z) < self.tolerance:
                   self.get_logger().info("switching to move to xy")
                   if self.target_x is not None and self.target_y is not None:
                       self.current_stage = "MOVE_TO_XY"
                   else:
                       setpoint_msg.position = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.lift_height]
               else:
                   setpoint_msg.position = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.lift_height]
                   # setpoint_msg.velocity = [0.0, 0.0, self.kp * error_z]




       elif self.current_stage == "MOVE_TO_XY":
           error_x = self.target_x - self.vehicle_local_position.x
           error_y = self.target_y - self.vehicle_local_position.y




           if math.sqrt(error_x**2 + error_y**2) < self.tolerance:
               self.get_logger().info(str(math.sqrt(error_x**2 + error_y**2)) + " switching to descend")
               self.current_stage = "DESCEND"
           else:
               setpoint_msg.position = [self.target_x, self.target_y, self.vehicle_local_position.z]
               # setpoint_msg.velocity = [self.kp * error_x, self.kp * error_y, 0.0]




       elif self.current_stage == "DESCEND":
           error_z = self.target_z - self.vehicle_local_position.z




           if abs(error_z) < self.tolerance:
               setpoint_msg.velocity = [0.0, 0.0, 0.0]  # Stop movement
               self.notify_state_machine()
           else:
               setpoint_msg.position = [self.target_x, self.target_y, self.target_z]
               # setpoint_msg.velocity = [0.0, 0.0, self.kp * error_z]




       # Publish the trajectory setpoint
       self.trajectory_setpoint_publisher.publish(setpoint_msg)




def main(args=None):
   rclpy.init(args=args)
   controller = UAVPositionController()
   rclpy.spin(controller)
   controller.destroy_node()
   rclpy.shutdown()




if __name__ == '__main__':
   main()


