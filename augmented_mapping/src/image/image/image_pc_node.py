import rclpy
from rclpy.node import Node
import numpy as np
# import pcl_conversions
import sensor_msgs.msg
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from scipy.spatial.transform import Rotation as R
from message_filters import Subscriber, TimeSynchronizer
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import transformations as tf



class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')
        self.bridge = CvBridge()


        # Create subscribers 
        self.depth_sub = Subscriber(self, Image, '/depth_camera') 
        self.pose_sub = Subscriber(self, Odometry, '/camera/odometry') 
        self.vehicle_position_sub = self.create_subscription(PoseStamped, '/vehicle/original_pose', self.vehicle_position_callback, 1)
        
        # Synchronize messages 
        self.ts = TimeSynchronizer([self.depth_sub, self.pose_sub], 10) 
        self.ts.registerCallback(self.depthPoseCallback)
        
        # PointCloud2 publisher
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/depth_camera/pointcloud', 10)


        self.fx = 432.4960
        self.fy = 324.372031
        self.cx = 320
        self.cy = 240

        self.k_depth_scaling_factor_ = 3413.28  # Scaling factor for depth
        self.depth_filter_margin_ = 2  # Depth filter margin
        self.depth_filter_maxdist_ = 10.0  # Max depth distance
        self.depth_filter_mindist_ = 0.2  # Min depth distance
        self.skip_pixel_ = 2

        self.camera_q_ = None  
        self.camera_pos_ = None
        self.depth_image_ = None
        self.depth_image_msg = None

        self.vehicle_position = None

    def vehicle_position_callback(self, vehicle_position):
        self.vehicle_position = vehicle_position

    def convert_pixel_coordinates(self, u_camera, v_camera):
        width_camera = 640
        height_camera = 480
        fov_camera = 1.204

        width_depth = 640
        height_depth = 480
        fov_depth = 1.274

        # Step 1: Convert from pixel to normalized image coordinates (camera)
        x_norm_camera = (2 * u_camera / width_camera) - 1
        y_norm_camera = 1 - (2 * v_camera / height_camera)
        # Step 2: Adjust for FOV differences
        x_norm_depth = x_norm_camera * (fov_camera / fov_depth)
        y_norm_depth = y_norm_camera * (fov_camera / fov_depth)  # Assuming same aspect ratio
        
        # Step 3: Convert back to pixel coordinates (depth camera)
        u_depth = int((x_norm_depth + 1) / 2 * width_depth)
        v_depth = int((1 - y_norm_depth) / 2 * height_depth)
        
        return u_depth, v_depth

    def depthPoseCallback(self, depth_image_msg, pose_msg):
        self.depth_image_msg = depth_image_msg
        # Update camera position
        self.camera_pos_ = np.array([
            pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
            pose_msg.pose.pose.position.z
        ])


        # Update camera orientation
        self.camera_q_ = np.array([
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w
        ])

        # Convert the image using CvBridge
        cv_ptr = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

        # Scale depth image if encoding is TYPE_32FC1
        
        cv_ptr = cv_ptr.astype(np.uint16) * self.k_depth_scaling_factor_

        self.depth_image_ = cv_ptr

        # Process the depth image and update the point cloud
        t1 = self.get_clock().now()
        self.process_depth_image(depth_image_msg)
        t2 = self.get_clock().now()


        # Time logging (in seconds)
        fuse_time = (t2 - t1).nanoseconds / 1e9
        self.get_logger().info(f'Fusion time: {fuse_time:.6f} seconds')

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


    def process_depth_image(self, depth_image_msg):
        # x,y = self.convert_pixel_coordinates(303.0, 157.0)
        u,v = 320, 180
        self.get_logger().info(str(u) + " " + str(v))
        # Convert depth image message to OpenCV format
        depth_image = self.depth_image_

        proj_points_cnt = 0
        rows, cols = depth_image.shape
        # camera_r = self.camera_q_.as_matrix()
        inv_factor = 1.0 / self.k_depth_scaling_factor_
        
        depth = depth_image[u,v]

        z = depth / self.k_depth_scaling_factor_
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        pt_cur = np.array([x,y,z])

        pt_cur = pt_cur.reshape(-1, 3)
        pt_cur = np.append(pt_cur, [[1]], axis=1)
        t_matrix = self.pose_to_transform(self.camera_pos_, self.camera_q_)

        pt_world = np.matmul(pt_cur, t_matrix)
        pt_world = pt_world[:, :3]
        self.get_logger().info(str(pt_world))

        # vehicle_r = vehicle_q_.as_matrix()

        # point_cloud = []

        # # Iterate over the depth image
        # for v in range(self.depth_filter_margin_, rows - self.depth_filter_margin_, self.skip_pixel_):
        #     for u in range(self.depth_filter_margin_, cols - self.depth_filter_margin_, self.skip_pixel_):
        #         depth = depth_image[v, u] * inv_factor


        #         # Apply depth filtering
        #         if depth_image[v, u] == 0 or depth > self.depth_filter_maxdist_:
        #             depth = self.depth_filter_maxdist_
        #         elif depth < self.depth_filter_mindist_:
        #             continue


        #         # Convert to 3D point in the camera frame
        #         pt_cur = np.array([
        #             (u - self.cx) * depth / self.fx,
        #             (v - self.cy) * depth / self.fy,
        #             depth
        #         ])


        #         # Transform to world frame
        #         pt_world = camera_r @ pt_cur + self.camera_pos_
                
                
                
        #         pt_world2 = vehicle_r @ pt_cur + vehicle_pos_



        #         # Add the point to the point cloud list
        #         point_cloud.append([pt_world[0], pt_world[1], pt_world[2]])
        #         proj_points_cnt += 1

        #         if (abs(v - x) <= 1 and abs(u - y) <= 1):
        #             self.get_logger().info(str(v) + " " + str(u) + " " + str(pt_world[1]) + " " + str(pt_world[0]) + " " + str(pt_world[2]))
        #             self.get_logger().info(str(v) + " " + str(u) + " " + str(pt_world2[0]) + " " + str(pt_world2[1]) + " " + str(pt_world2[2]))


        # Store the processed point cloud
        # self.point_cloud_ = point_cloud


        # Publish the point cloud
        # self.publish_depth()

    def publish_depth(self):
        # Create a PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "x500_depth_0/OakD-Lite/base_link/StereoOV7251"


        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Convert point cloud (list of [x, y, z]) to PointCloud2 message
        cloud_msg = pc2.create_cloud(header, fields, self.point_cloud_)


        # Publish the point cloud
        self.pointcloud_pub.publish(cloud_msg)



def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloud()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()