import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat as quaternion_from_euler
from collections import deque
import transformations as tf
# import scipy.spatial.transform as stf
import numpy as np

# Pose of StereoOV7251 sensor relative to OakD-Lite frame
T_camera_world = np.array([
   [1, 0, 0, 0.012],
   [0, 1, 0, 0.03],
   [0, 0, 1, 0.242],
   [0, 0, 0, 1]
])

# Pose of OakD-Lite frame relative to x500_depth frame
T_uav_world = np.array([
   [1, 0, 0, 0],
   [0, 1, 0, 0],
   [0, 0, 1, 0.24],
   [0, 0, 0, 1]
])


T_uav_world_inv = np.linalg.inv(T_uav_world)

class Perception(Node):
    """Node for controlling a vehicle in offboard mode using velocity control."""

    def __init__(self) -> None:
        super().__init__('attitude_control')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.odometry_topic_publisher = self.create_publisher(
            Odometry, '/vehicle/odometry', qos_profile)
        self.sensor_pose_publisher = self.create_publisher(
            PoseStamped, '/vehicle/sensor_pose', qos_profile)

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        # Initialize variables
        self.T_cam_uav = np.matmul(T_uav_world_inv, T_camera_world)
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz timer
        self.altitude = -5.0  # Target altitude in meters
        self.start_time = self.get_clock().now()
        self.last_time = self.start_time

    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_odometry = vehicle_odometry
                
    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def ned_to_enu(self, T_ned):
        T = np.array([[0, 1, 0, 0],
                        [1, 0, 0, 0],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1]])
        return np.matmul(T, T_ned)


    def pose_to_transform(self, position, quaternion):
        """
        Convert position and quaternion to a 4x4 transformation matrix.

        :param position: A list or array of x, y, z coordinates.
        :param quaternion: A list or array of x, y, z, w quaternion components.
        :return: A 4x4 transformation matrix.
        """
        transform_matrix = np.eye(4)
        r = tf.quaternion_matrix(quaternion)
        transform_matrix[:3, :3] = r[:3, :3]
        transform_matrix[:3, 3] = position
        return transform_matrix

    def transform_to_quaternion(self, transform_matrix):
        """
        Extract quaternion from a 4x4 transformation matrix.

        :param transform_matrix: A 4x4 transformation matrix.
        :return: A quaternion [x, y, z, w].
        """
        quaternion = tf.quaternion_from_matrix(transform_matrix)
        return quaternion


    # Create the transformation matrix
    def timer_callback(self) -> None:
        """Callback function for the timer."""

        odom = Odometry()

        # Header
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        position = self.vehicle_odometry.position
        # Quanternion in x,y,z,w
        quaternion = [self.vehicle_odometry.q[3], self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2]]

        t_matrix = self.pose_to_transform(position, quaternion)
        t_matrix_ENU = self.ned_to_enu(t_matrix)

        new_quaternion = self.transform_to_quaternion(t_matrix_ENU)
        new_position = t_matrix_ENU[:3, 3]

        # Pose
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = float(new_position[0])
        odom.pose.pose.position.y = float(new_position[1])
        odom.pose.pose.position.z = float(new_position[2])

        odom.pose.pose.orientation.x = float(new_quaternion[0])
        odom.pose.pose.orientation.y = float(new_quaternion[1])
        odom.pose.pose.orientation.z = float(new_quaternion[2])
        odom.pose.pose.orientation.w = float(new_quaternion[3])

        # Twist
        odom.twist.twist = Twist()
        odom.twist.twist.linear.x = float(self.vehicle_odometry.velocity[1])
        odom.twist.twist.linear.y = float(self.vehicle_odometry.velocity[0])
        odom.twist.twist.linear.z = -float(self.vehicle_odometry.velocity[2])
        odom.twist.twist.angular.x = -float(self.vehicle_odometry.angular_velocity[1])
        odom.twist.twist.angular.y = float(self.vehicle_odometry.angular_velocity[0])
        odom.twist.twist.angular.z = float(self.vehicle_odometry.angular_velocity[2])

        camera_offset = [0.12, 0.03, 0.002]

        camera_offset_T = np.eye(4)
        camera_offset_T[:3, 3] = camera_offset

        # self.get_logger().info(str(t_matrix_ENU))

        T_camera_ENU = np.matmul(camera_offset_T, t_matrix_ENU)

        cam_position = T_camera_ENU[:3, 3]
        cam_orientation = self.transform_to_quaternion(T_camera_ENU)

        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"  # Adjust frame_id as per your setup
        pose_msg.pose.position.x = cam_position[0]
        pose_msg.pose.position.y = cam_position[1]
        pose_msg.pose.position.z = cam_position[2]
        pose_msg.pose.orientation.x = cam_orientation[0]
        pose_msg.pose.orientation.y = cam_orientation[1]
        pose_msg.pose.orientation.z = cam_orientation[2]
        pose_msg.pose.orientation.w = cam_orientation[3]

        self.odometry_topic_publisher.publish(odom)
        self.sensor_pose_publisher.publish(pose_msg)

        self.get_logger().info("odom " + str(new_quaternion))
        self.get_logger().info("sensor " + str(cam_orientation))

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    perception_node = Perception()
    rclpy.spin(perception_node)
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    # def quaternion_from_matrix(self, matrix):
    #     # Convert rotation matrix to quaternion
    #     q = np.empty((4,))
    #     M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    #     t = np.trace(M)
        
    #     if t > M[3, 3]:
    #         q[0] = t
    #         q[3] = M[1, 0] - M[0, 1]
    #         q[2] = M[0, 2] - M[2, 0]
    #         q[1] = M[2, 1] - M[1, 2]
    #     else:
    #         i, j, k = 0, 1, 2
    #         if M[1, 1] > M[0, 0]:
    #             i, j, k = 1, 2, 0
    #         if M[2, 2] > M[i, i]:
    #             i, j, k = 2, 0, 1
    #         t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
    #         q[i] = t
    #         q[j] = M[i, j] + M[j, i]
    #         q[k] = M[k, i] + M[i, k]
    #         q[3] = M[k, j] - M[j, k]
        
    #     q *= 0.5 / np.sqrt(t * M[3, 3])
    #     return q