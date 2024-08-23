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

# import boxing box ros message
from vision_msgs.msg import BoundingBox2D, Detection2DArray, Detection2D



class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')
        self.bridge = CvBridge()


        # Create subscribers 
        self.depth_sub = Subscriber(self, Image, '/depth_camera') 
        self.pose_sub = Subscriber(self, Odometry, '/camera/odometry') 
        self.vehicle_position_sub = self.create_subscription(PoseStamped, '/vehicle/original_pose', self.vehicle_position_callback, 1)

        self.bbox_sub = self.create_subscription(Detection2DArray, '/yolov7_detections', self.bbox_callback, 1)
        
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

        self.point_cloud_pub = None

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
        self.get_logger().info("depth image shape 1 " + str(self.depth_image_.shape))

        # convert depth image to point cloud using depth image, camera intrinsics, camera pose, and scaling factor
        # get z values from depth image
        z_values = cv_ptr.flatten()
        # get x, y values from pixel coordinates
        x, y = np.meshgrid(np.arange(0, cv_ptr.shape[1]), np.arange(0, cv_ptr.shape[0]))
        x = x.flatten()
        y = y.flatten()
        # get the pixel coordinates
        u = x
        v = y
        # convert pixel coordinates to depth camera coordinates
        #u, v = self.convert_pixel_coordinates(u, v)
        # get the depth values
        z = z_values
        # convert depth values to meters using the scaling factor
        z = z / self.k_depth_scaling_factor_
        
        # convert the pixel coordinates to camera coordinates using camera intrinsics
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        # convert the 3D coordinates to world coordinates
        # get the transformation matrix
        T = self.pose_to_transform(self.camera_pos_, self.camera_q_)
        # get the 3D coordinates in world frame
        point = np.array([x, y, z, np.ones_like(x)])
        point = np.matmul(T, point)
        point = point[:3]
        self.point_cloud_ = point.T
        # self.get_logger().info("point shapre 1 " + str(self.point_cloud_.shape))

        # Publish the point cloud
        # self.publish_depth()

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


    def bbox_callback(self, detection2d_msg):
        # self.get_logger().info("bbox callback called")
        # Get the bounding box coordinates
        if self.depth_image_ is not None:
            for detection2d in detection2d_msg.detections:
                bbox_msg = detection2d.bbox
                x1 = int(bbox_msg.center.position.x - bbox_msg.size_x / 2)   
                x2 = int(bbox_msg.center.position.x + bbox_msg.size_x / 2)
                y1 = int(bbox_msg.center.position.y - bbox_msg.size_y / 2)
                y2 = int(bbox_msg.center.position.y + bbox_msg.size_y / 2)
                # self.get_logger().info(str(x1) + " " + str(x2) + " " + str(y1) + " " + str(y2))

                # shrink the bounding box by 10 percent
                x1 = int(x1 + 0.1 * (x2 - x1))
                x2 = int(x2 - 0.1 * (x2 - x1))
                y1 = int(y1 + 0.1 * (y2 - y1))
                y2 = int(y2 - 0.1 * (y2 - y1))
                

                # Convert the pixel coordinates to depth camera coordinates
                u1, v1 = x1, y1
                u2, v2 = x2, y2


                # # Get the depth values
                # depth_values = []
                # for i in range(u1, u2):
                #     for j in range(v1, v2):
                #         depth_value = self.depth_image_[j, i]
                #         # convert depth value to meters using the scaling factor
                #         depth_value = depth_value / self.k_depth_scaling_factor_
                #         depth_values.append(depth_value)


                # # Get the average depth value
                # avg_depth = np.mean(depth_values)

                # Get the 3D coordinates of the bounding box
                # x_mid = int((x1 + x2) / 2)
                # y_mid = int((y1 + y2) / 2)
                # z_depth = self.depth_image_[y_mid, x_mid]

                # z = z_depth / self.k_depth_scaling_factor_
                # x = (x_mid - self.cx) * z / self.fx
                # y = (y_mid - self.cy) * z / self.fy
                # z_bbox = self.depth_image_[v1:v2, u1:u2]
                # z_vals = z_bbox / self.k_depth_scaling_factor_

                # x, y = np.meshgrid(np.arange(0, self.depth_image_.shape[1]), np.arange(0, self.depth_image_.shape[0])) 
                # Apply the bounding box to get the region of interest 
                # x_bbox = x[y1:y2, x1:x2] 
                # y_bbox = y[y1:y2, x1:x2]

                # x_cal = (x_bbox - self.cx) * z_vals / self.fx
                # y_cal = (y_bbox - self.cy) * z_vals / self.fy

                # T = self.pose_to_transform(self.camera_pos_, self.camera_q_)
                # # get the 3D coordinates in world frame
                # point_orig = np.array([x_cal, y_cal, z_vals, np.ones_like(x_cal)])
                # point_flat = point_orig.reshape(4, -1)
                # point_flat = np.matmul(T, point_flat)
                # point = point_flat[:3]
                # self.get_logger().info(str(point.shape))

                # depth_cam_pts = point.reshape(point_orig.shape[1], point_orig.shape[2], 3)

                # self.get_logger().info(str(depth_cam_pts))
                # self.get_logger().info(str(depth_cam_pts.shape))
                # self.get_logger().info("point cloud points")

                # self.point_cloud_pub = depth_cam_pts
                # self.publish_depth()


                # ros log info
                if self.point_cloud_ is not None:
                    height, width = self.depth_image_.shape
                    # self.get_logger().info("depth image shape 2 " + str(self.depth_image_.shape))

                    # Reshape each component of the point cloud back to the original image shape
                    x_reshaped = self.point_cloud_[:, 0].reshape(height, width)
                    y_reshaped = self.point_cloud_[:, 1].reshape(height, width)
                    z_reshaped = self.point_cloud_[:, 2].reshape(height, width)

                    x_bbox = x_reshaped[v1:v2, u1:u2] 
                    y_bbox = y_reshaped[v1:v2, u1:u2] 
                    z_bbox = z_reshaped[v1:v2, u1:u2] 

                    # Stack the components back into the point cloud format (3D array) 
                    point_bbox = np.stack((x_bbox, y_bbox, z_bbox), axis=-1)

                    rows, cols, _ = point_bbox.shape
                    center_row = rows // 2
                    center_col = cols // 2

                    center_coordinates = point_bbox[center_row, center_col]
                    self.get_logger().info(str(center_coordinates))

                    self.point_cloud_pub = point_bbox
                    self.publish_depth()

                    # self.get_logger().info(str(point_bbox))
                    # self.get_logger().info(str(point_bbox.shape))

                

                    # self.get_logger().info(str(x_mid) + " " + str(y_mid))
                    # pt_world2 = self.point_cloud_[int(x_mid * 640 + y_mid)]
                    # pt_world3 = self.point_cloud_[int(x_mid * 480 + y_mid)]
                    # pt_world4 = self.point_cloud_[int(y_mid * 640 + x_mid)]
                    # pt_world5 = self.point_cloud_[int(y_mid * 480 + x_mid)]
                    # self.get_logger().info(str(pt_world))
                    # self.get_logger().info(str(pt_world2))
                    # self.get_logger().info(str(pt_world3))
                    # self.get_logger().info(str(pt_world4))
                    # self.get_logger().info(str(pt_world5))
        

    def publish_depth(self):
        # Create a PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"


        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Convert point cloud (list of [x, y, z]) to PointCloud2 message
        cloud_msg = pc2.create_cloud(header, fields, self.point_cloud_pub)


        # Publish the point cloud
        self.pointcloud_pub.publish(cloud_msg)



def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloud()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()








