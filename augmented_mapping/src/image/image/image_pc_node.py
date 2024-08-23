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
        self.position_pub = self.create_publisher(PoseStamped, '/target_pos', 10)


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

        # convert depth image to point cloud using depth image, camera intrinsics, camera pose, and scaling factor
        # get z values from depth imageqos_profile
        z_values = cv_ptr.flatten()

        # get x, y values from pixel coordinates
        x, y = np.meshgrid(np.arange(0, cv_ptr.shape[1]), np.arange(0, cv_ptr.shape[0]))
        x = x.flatten()
        y = y.flatten()
        
        # get the pixel coordinates
        u = x
        v = y

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

    def pose_to_transform(self, position, quaternion):
        """
        Convert position and quaternion to a 4x4 transformation matrix.qos_profile

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

        # Get the bounding box coordinates
        if self.depth_image_ is not None:
            for detection2d in detection2d_msg.detections:
                bbox_msg = detection2d.bbox
                x1 = int(bbox_msg.center.position.x - bbox_msg.size_x / 2)   
                x2 = int(bbox_msg.center.position.x + bbox_msg.size_x / 2)
                y1 = int(bbox_msg.center.position.y - bbox_msg.size_y / 2)
                y2 = int(bbox_msg.center.position.y + bbox_msg.size_y / 2)

                # shrink the bounding box by 10 percent
                x1 = int(x1 + 0.1 * (x2 - x1))
                x2 = int(x2 - 0.1 * (x2 - x1))
                y1 = int(y1 + 0.1 * (y2 - y1))
                y2 = int(y2 - 0.1 * (y2 - y1))
                

                # Convert the pixel coordinates to depth camera coordinates
                u1, v1 = x1, y1
                u2, v2 = x2, y2


                # ros log info
                if self.point_cloud_ is not None:
                    height, width = self.depth_image_.shape

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
                    
                    position_msg = PoseStamped()
                    position_msg.pose.position.x = center_coordinates[1]
                    position_msg.pose.position.y = center_coordinates[0]
                    position_msg.pose.position.z = center_coordinates[2]

                    self.position_pub.publish(position_msg)

                    self.point_cloud_pub = point_bbox
                    self.publish_depth()
        

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








