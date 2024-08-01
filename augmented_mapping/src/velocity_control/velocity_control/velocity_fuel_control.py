import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint, TrajectorySetpoint
from geometry_msgs.msg import Quaternion, PoseStamped, Pose
from transforms3d.euler import euler2quat as quaternion_from_euler
from quadrotor_msgs.msg import SO3Command
from collections import deque
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

class PID:
    def __init__(self, Kp, Ki, Kd, integral_window_size=500):
        """
        Initialize the PID controller with given gains and integral window size.

        :param Kp: Proportional gain
        :param Ki: Integral gain
        :param Kd: Derivative gain
        :param integral_window_size: Number of recent errors to consider for the integral term
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_window_size = integral_window_size
        self.integral_errors = deque(maxlen=integral_window_size)
        self.previous_error = 0

    def update(self, measurement, setpoint, dt):
        """
        Update the PID controller with the current measurement and setpoint.

        :param measurement: Current measurement
        :param setpoint: Desired setpoint value
        :param dt: Time interval since the last update
        :return: Control output
        """
        error = setpoint - measurement
        self.integral_errors.append(error * dt)
        integral = sum(self.integral_errors)
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        output = self.Kp * error + self.Ki * integral + self.Kd * derivative
        return output

class VelocityControl(Node):
    """Node for controlling a vehicle in offboard mode using velocity control."""

    def __init__(self) -> None:
        super().__init__('velocity_manual_control')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.pos_vel_subscriber = self.create_subscription(
            Float32MultiArray, '/pos_vel_cmd', self.pos_vel_callback, 50)

        # Create PID controllers
        self.pid_x = PID(2.2, 0.1, 0.0)
        self.pid_y = PID(0.9, 0.1, 0.0)
        self.pid_z = PID(0.9, 0.1, 0.0)
        self.pid_yaw = PID(2.8, 0.1, 0.0)


        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz timer
        self.start_time = self.get_clock().now()
        self.last_time = self.start_time
        self.pos_vel = None
        self.takeoff = True

        self.arm_status = False

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
                

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def normalize_angle(self, angle):
        """ Normalize an angle to the range [-π, π) """
        if angle >= np.pi:
            angle -= 2 * np.pi  # Shift to [-π, π)
        return angle


    def pos_vel_callback(self, pv_msg):
        """Callback function for vehicle_status topic subscriber."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        desired_pos_x = pv_msg.data[1]
        desired_pos_y = pv_msg.data[0]
        desired_pos_z = -pv_msg.data[2]
        transformed_angle = -pv_msg.data[3] + 0.5 * np.pi
        desired_yaw = self.normalize_angle(transformed_angle)

        desired_vel_x = pv_msg.data[5]
        desired_vel_y = pv_msg.data[4]
        desired_vel_z = -pv_msg.data[6]
        desired_yawdot = -pv_msg.data[7]

        error_x = desired_pos_x - self.vehicle_local_position.x
        error_y = desired_pos_y - self.vehicle_local_position.y
        error_z = desired_pos_z - self.vehicle_local_position.z
        actual_yaw = self.vehicle_local_position.heading

        if (desired_yaw - self.vehicle_local_position.heading > math.pi):
            desired_yaw -= 2 * math.pi
        elif (desired_yaw - self.vehicle_local_position.heading < -math.pi):
            actual_yaw -= 2 * math.pi
        
        error_yaw =  desired_yaw - actual_yaw
        
        velocity_x = self.pid_x.update(self.vehicle_local_position.x, desired_pos_x, dt) + desired_vel_x
        velocity_y = self.pid_y.update(self.vehicle_local_position.y, desired_pos_y, dt) + desired_vel_y
        velocity_z = self.pid_z.update(self.vehicle_local_position.z, desired_pos_z, dt) + desired_vel_z
        yaw_rate = self.pid_yaw.update(actual_yaw, desired_yaw, dt) + desired_yawdot
        self.pos_vel = [velocity_x, velocity_y, velocity_z, yaw_rate]

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

    def land(self):
        """Switch to land mode."""           
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
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

    def timer_callback(self) -> None:
        """Callback function for the timer."""

        if self.offboard_setpoint_counter == 0:
            self.engage_offboard_mode()
            self.arm()
        self.offboard_setpoint_counter += 1

        self.publish_offboard_control_heartbeat_signal()
        msg = TrajectorySetpoint()

        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan') 

        if self.takeoff == True:
            if self.vehicle_local_position.z > -4.0:
                msg.velocity = [-0.0, 0.0, -1.0]
                msg.yawspeed = 0.0
            else:
                self.takeoff = False
                self.get_logger().info("Reached 4 meters")
        else:
            if self.pos_vel is not None:
                # self.get_logger().info("pos vel is not none")
                msg.velocity = [self.pos_vel[0], self.pos_vel[1], self.pos_vel[2]]
                msg.yawspeed = self.pos_vel[3]
            else:
                msg.velocity = [0.0, 0.0, 0.0]
                msg.yawspeed = 0.0

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    velocity_control = VelocityControl()
    rclpy.spin(velocity_control)
    velocity_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()