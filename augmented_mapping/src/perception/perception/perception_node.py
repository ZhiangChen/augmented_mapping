import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat as quaternion_from_euler
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from collections import deque
import transformations as tf
import numpy as np


class Perception(Node):
   """Node for controlling a vehicle in offboard mode using velocity control."""


   def __init__(self) -> None:
       super().__init__('perception_node')


       # Configure QoS profile for publishing and subscribing
       qos_profile = QoSProfile(
           reliability=ReliabilityPolicy.BEST_EFFORT,
           durability=DurabilityPolicy.TRANSIENT_LOCAL,
           history=HistoryPolicy.KEEP_LAST,
           depth=1
       )


       # Create publishers
       self.odometry_topic_publisher = self.create_publisher(
           Odometry, '/vehicle/odometry', 1)
       self.sensor_pose_publisher = self.create_publisher(
           PoseStamped, '/vehicle/sensor_pose', 1)
       self.orig_pose_publisher = self.create_publisher(
           PoseStamped, '/vehicle/original_pose', 1)
       self.camera_info_publisher = self.create_publisher(
           CameraInfo, '/camera_info', 1)


       # Create subscribers
       self.vehicle_odometry_subscriber = self.create_subscription(
           VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)


       self.image_subscriber = self.create_subscription(
           Image, '/camera', self.camera_callback, 1)


       # Initialize variables
       self.offboard_setpoint_counter = 0
       self.vehicle_odometry = VehicleOdometry()
       self.altitude = -5.0  # Target altitude in meters
       self.start_time = self.get_clock().now()
       self.last_time = self.start_time


   def vehicle_odometry_callback(self, vehicle_odometry):
       """Callback function for vehicle_local_position topic subscriber."""
       self.vehicle_odometry = vehicle_odometry
       odom = Odometry()


       # Header
       odom.header = Header()
       odom.header.stamp = self.get_clock().now().to_msg()
       odom.header.frame_id = 'base_link'
       odom.child_frame_id = 'odom'


       position = [self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], self.vehicle_odometry.position[2] + 1.5]
       # Quanternion in w, x, y, z
       quaternion = [self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3]]


       t_matrix = self.pose_to_transform(position, quaternion)
       t_matrix_ENU = self.ned_to_enu(t_matrix)


       new_quaternion = self.transform_to_quaternion(t_matrix_ENU)
       new_position = t_matrix_ENU[:3, 3]


       # Pose
       odom.pose.pose = Pose()
       odom.pose.pose.position.x = float(new_position[0])
       odom.pose.pose.position.y = float(new_position[1])
       odom.pose.pose.position.z = float(new_position[2])


       odom.pose.pose.orientation.x = float(new_quaternion[1])
       odom.pose.pose.orientation.y = float(new_quaternion[2])
       odom.pose.pose.orientation.z = float(new_quaternion[3])
       odom.pose.pose.orientation.w = float(new_quaternion[0])


       # Twist
       odom.twist.twist = Twist()
       odom.twist.twist.linear.x = float(self.vehicle_odometry.velocity[1])
       odom.twist.twist.linear.y = float(self.vehicle_odometry.velocity[0])
       odom.twist.twist.linear.z = -float(self.vehicle_odometry.velocity[2])
       odom.twist.twist.angular.x = float(self.vehicle_odometry.angular_velocity[1])
       odom.twist.twist.angular.y = float(self.vehicle_odometry.angular_velocity[0])
       odom.twist.twist.angular.z = -float(self.vehicle_odometry.angular_velocity[2])


       camera_offset = [0.15, 0.03, 0.002]


       camera_offset_T = np.eye(4)
       camera_offset_T[:3, 3] = camera_offset
       # Camera rotation
       camera_rotation = np.array([[0, 0, 1,0 ], [-1, 0, 0, 0], [0, -1, 0, 0], [0,0,0,1]])
       camera_offset_T[:3, :3] = camera_rotation[:3, :3]


       # self.get_logger().info(str(t_matrix_ENU))


       T_camera_ENU = np.matmul(t_matrix_ENU, camera_offset_T)


       cam_position = T_camera_ENU[:3, 3]
       cam_orientation = self.transform_to_quaternion(T_camera_ENU)


       # Create a PoseStamped message
       pose_msg = PoseStamped()
       pose_msg.header.stamp = self.get_clock().now().to_msg()
       pose_msg.header.frame_id = "world"  # Adjust frame_id as per your setup
       pose_msg.pose.position.x = cam_position[0]
       pose_msg.pose.position.y = cam_position[1]
       pose_msg.pose.position.z = cam_position[2]
       pose_msg.pose.orientation.x = cam_orientation[1]
       pose_msg.pose.orientation.y = cam_orientation[2]
       pose_msg.pose.orientation.z = cam_orientation[3]
       pose_msg.pose.orientation.w = cam_orientation[0]


       orig_pose = PoseStamped()


       orig_pose.header.stamp = self.get_clock().now().to_msg()
       orig_pose.header.frame_id = "world"  # Adjust frame_id as per your setup
       orig_pose.pose.position.x = float(self.vehicle_odometry.position[0])
       orig_pose.pose.position.y = float(self.vehicle_odometry.position[1])
       orig_pose.pose.position.z = float(self.vehicle_odometry.position[2])


       orig_pose.pose.orientation.x = float(self.vehicle_odometry.q[1])
       orig_pose.pose.orientation.y = float(self.vehicle_odometry.q[2])
       orig_pose.pose.orientation.z = float(self.vehicle_odometry.q[3])
       orig_pose.pose.orientation.w = float(self.vehicle_odometry.q[0])


       self.odometry_topic_publisher.publish(odom)
       self.orig_pose_publisher.publish(orig_pose)
       self.sensor_pose_publisher.publish(pose_msg)


   def camera_callback(self, image):
       cam_info = CameraInfo()


       cam_info.header = image.header
       cam_info.height = 480
       cam_info.width = 640
       cam_info.distortion_model = "plumb_bob"


       cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
       cam_info.k = [465.7411193847656, 0.0, 320.0, 0.0, 465.7411766052246, 240.0, 0.0, 0.0, 1.0]
       cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
       cam_info.p = [465.7411193847656, 0.0, 320.0, 0.0, 0.0, 465.7411766052246, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
      
       cam_info.binning_x = 0
       cam_info.binning_y = 0


       cam_info.roi = RegionOfInterest()


       cam_info.roi.x_offset = 0
       cam_info.roi.y_offset = 0
       cam_info.roi.height = 0
       cam_info.roi.width = 0
       cam_info.roi.do_rectify = False


       self.camera_info_publisher.publish(cam_info)




              
   def vehicle_status_callback(self, vehicle_status):
       """Callback function for vehicle_status topic subscriber."""
       self.vehicle_status = vehicle_status


   def ned_to_enu(self, T_ned):
       T2 = np.array([[0, 1, 0, 0],
                       [1, 0, 0, 0],
                       [0, 0, -1, 0],
                       [0, 0, 0, 1]])
       T1 = np.array([[1, 0, 0,0 ], [0, -1, 0, 0], [0, 0, -1, 0], [0,0,0,1]])
       return np.matmul(np.matmul(T2, T_ned), T1)
       # return np.matmul(T2, T_ned)




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


def main(args=None) -> None:
   print('Starting perception node...')
   rclpy.init(args=args)
   perception_node = Perception()
   rclpy.spin(perception_node)
   perception_node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

