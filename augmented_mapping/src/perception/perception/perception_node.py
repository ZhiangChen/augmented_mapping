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
import numpy as np

# Pose of StereoOV7251 sensor relative to OakD-Lite frame
T_camera_sensor = np.array([
   [1, 0, 0, 0.01233],
   [0, 1, 0, -0.03],
   [0, 0, 1, 0.01878],
   [0, 0, 0, 1]
])

# Pose of OakD-Lite frame relative to x500_depth frame
T_uav_camera = np.array([
   [1, 0, 0, 0.12],
   [0, 1, 0, 0.03],
   [0, 0, 1, 25.8],
   [0, 0, 0, 1]
])

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
           VehicleOdometry, '/fmu/out/vehicle_local_position', self.vehicle_odometry_callback, qos_profile)

       # Initialize variables
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

   def create_translation_matrix(position, quaternion):
   # Create the translation matrix
       translation_matrix = tf.translation_matrix(position)
      
       # Create the rotation matrix from the quaternion
       rotation_matrix = tf.quaternion_matrix(quaternion)
      
       # Combine the translation and rotation matrices into a single transformation matrix
       transformation_matrix = np.dot(translation_matrix, rotation_matrix)
      
       return transformation_matrix
  
   def quaternion_from_matrix(matrix):
       # Convert rotation matrix to quaternion
       q = np.empty((4,))
       M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
       t = np.trace(M)
      
       if t > M[3, 3]:
           q[0] = t
           q[3] = M[1, 0] - M[0, 1]
           q[2] = M[0, 2] - M[2, 0]
           q[1] = M[2, 1] - M[1, 2]
       else:
           i, j, k = 0, 1, 2
           if M[1, 1] > M[0, 0]:
               i, j, k = 1, 2, 0
           if M[2, 2] > M[i, i]:
               i, j, k = 2, 0, 1
           t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
           q[i] = t
           q[j] = M[i, j] + M[j, i]
           q[k] = M[k, i] + M[i, k]
           q[3] = M[k, j] - M[j, k]
      
       q *= 0.5 / np.sqrt(t * M[3, 3])
       return q


   # Create the transformation matrix
   def timer_callback(self) -> None:
       """Callback function for the timer."""

       odom = Odometry()

       # Header
       odom.header = Header()
       odom.header.stamp = self.get_clock().now().to_msg()
       odom.header.frame_id = 'odom'
       odom.child_frame_id = 'base_link'

       # Pose
       odom.pose.pose = Pose()
       odom.pose.pose.position = Point(*self.vehicle_odometry.position)
       odom.pose.pose.orientation = Quaternion(*self.vehicle_odometry.q)

       # Twist
       odom.twist.twist = Twist()
       odom.twist.twist.linear = Vector3(*self.vehicle_odometry.velocity)
       odom.twist.twist.angular = Vector3(*self.vehicle_odometry.angular_velocity)

       T_uav_world = self.create_translation_matrix(self.vehicle_odometry.position, self.vehicle_odometry.q)
       T_camera_uav = np.dot(T_uav_camera, T_camera_sensor)
       T_camera_world = np.dot(T_uav_world, T_camera_uav)

       # Extract position (translation vector) from the transformation matrix
       position = T_camera_world[:3, 3]
      
       # Extract orientation (rotation matrix to quaternion) from the transformation matrix
       rotation_matrix = T_camera_world[:3, :3]
       orientation = self.quaternion_from_matrix(rotation_matrix)

       # Create a PoseStamped message
       pose_msg = PoseStamped()
       pose_msg.header.stamp = self.get_clock().now().to_msg()
       pose_msg.header.frame_id = "world"  # Adjust frame_id as per your setup
       pose_msg.pose.position = Point(*position)
       pose_msg.pose.orientation = Quaternion(*orientation)

       self.sensor_pose_publisher.publish(pose_msg)

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    perception_node = Perception()
    rclpy.spin(perception_node)
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
