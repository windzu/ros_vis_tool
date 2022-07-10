from turtle import distance
import numpy as np
from scipy.spatial.transform import Rotation as R

# ros
import rospy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2

from geometry_msgs.msg import Point

# from visualization_msgs import ImageMarker
from visualization_msgs.msg import ImageMarker


# pypcd
from pypcd import pypcd


class ROSVisualizer:
    def __init__(
        self,
        lidar_frame_id_list=None,
        lidar_topic_suffix="",
        camera_frame_id_list=None,
        camera_topic_suffix="",
        static_tf_info_hub=None,
    ):
        self.lidar_frame_id_list = lidar_frame_id_list
        self.lidar_topic_suffix = lidar_topic_suffix
        self.camera_frame_id_list = camera_frame_id_list
        self.camera_topic_suffix = camera_topic_suffix
        self.static_tf_info_hub = static_tf_info_hub

    # def _lidar_subscibers_init(self):
    #     # subscriber all lidar topics
    #     for lidar_frame_id in self.lidar_frame_id_list:
    #         lidar_topic = lidar_frame_id + self.lidar_topic_suffix
    #         rospy.Subscriber(lidar_topic, PointCloud2, self._lidar_callback)
    #     pass

    # def _camera_subscibers_init(self):
    #     pass

    def start(self):
        # # create a publisher for image marker
        # def _create_image_marker_publisher(self, camera_frame_id):
        #     image_marker_topic = camera_frame_id + self.camera_topic_suffix
        #     return rospy.Publisher(image_marker_topic, ImageMarker, queue_size=10)
        # test

        lidar_frame_id = "LIDAR_TOP"
        camera_frame_id = "CAM_FRONT"
        covert_mode = "3DPOINTS"
        image_marker_topic = camera_frame_id + "/image_marker"
        image_marker_publisher = rospy.Publisher(image_marker_topic, ImageMarker, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            image_marker = self.lidar_to_camera(lidar_frame_id, camera_frame_id, covert_mode)
            image_marker_publisher.publish(image_marker)
            rate.sleep()

    def lidar_to_camera(self, lidar_frame_id, camera_frame_id, covert_mode):
        tf_stamped = self.get_tf_info(lidar_frame_id, camera_frame_id)
        camera_info = self.get_camera_info(camera_frame_id)

        if covert_mode == "3DPOINTS":
            lidar_topic = lidar_frame_id + self.lidar_topic_suffix
            pointcloud2 = rospy.wait_for_message(lidar_topic, PointCloud2, timeout=None)
            image_marker = self._convert_3dpoints_to_pixel(
                tf_stamped=tf_stamped, camera_info=camera_info, pointcloud2=pointcloud2
            )
            return image_marker
        elif covert_mode == "3DBBOX":
            pass
        else:
            pass

    @staticmethod
    def _convert_3dpoints_to_pixel(tf_stamped, camera_info, pointcloud2):

        pc = pypcd.PointCloud.from_msg(pointcloud2)
        x = pc.pc_data["x"].flatten()
        y = pc.pc_data["y"].flatten()
        z = pc.pc_data["z"].flatten()
        points = np.zeros((4, x.shape[0]))
        points[0, :] = x
        points[1, :] = y
        points[2, :] = z
        points[3, :] = 1.0

        # transform points to camera frame
        transform_matrix = np.eye(4)
        # convert tf_stamped to matrix
        r = R.from_quat(
            (
                tf_stamped.transform.rotation.x,
                tf_stamped.transform.rotation.y,
                tf_stamped.transform.rotation.z,
                tf_stamped.transform.rotation.w,
            )
        )
        transform_matrix[:3, :3] = r.as_matrix()
        transform_matrix[:3, 3] = (
            tf_stamped.transform.translation.x,
            tf_stamped.transform.translation.y,
            tf_stamped.transform.translation.z,
        )

        # transform points
        points = np.dot(transform_matrix, points)
        # convert points to pixel
        # convert camera_info.P from float64 tuple to numpy array
        camera_info.P = np.array(camera_info.P).astype(np.float32)
        camera_projection_matrix = camera_info.P.reshape(3, 4)
        points = np.dot(camera_projection_matrix, points)

        # find points in front of camera
        # find points[2, :]<0 index for mask
        mask = points[2, :] < 0
        points = points[:, ~mask]
        distance = points[2, :].flatten().tolist()
        points = (points / points[2, :]).astype(np.int32)

        # points out of camera range
        # find points[0, :]<0 or points[0, :]>=camera_info.width or points[1, :]<0 or points[1, :]>=camera_info.height index for mask
        mask = (
            (points[0, :] < 0)
            | (points[0, :] >= camera_info.width)
            | (points[1, :] < 0)
            | (points[1, :] >= camera_info.height)
        )
        points = points[:, ~mask]

        # create image marker
        image_marker = ImageMarker()
        image_marker.header.frame_id = camera_info.header.frame_id
        image_marker.header.stamp = rospy.Time.now()
        image_marker.ns = "points"
        image_marker.id = 0
        image_marker.type = ImageMarker.POINTS
        image_marker.action = image_marker.ADD
        image_marker.scale = 2
        image_marker.lifetime = rospy.Duration(0.0)

        for i in range(points.shape[1]):
            point = Point()
            point.x = points[0, i]
            point.y = points[1, i]
            point.z = 0.0
            image_marker.points.append(point)
            color = ColorRGBA()
            color.r = 0.0
            color.g = 1.0 * (distance[i] / 100)
            color.b = 1.0 * (distance[i] / 100)
            color.a = 1.0

            # test
            print("distance : ", distance[i])
            image_marker.outline_colors.append(color)
        # image_marker.points = [temp_points.x=point[0], point[1], 0) for point in points.T]

        return image_marker

    # points = tf_stamped.transform_points(points)

    def get_tf_info(self, parent_frame_id, child_frame_id):
        return self.static_tf_info_hub.get_tf_info(parent_frame_id=parent_frame_id, child_frame_id=child_frame_id)

    def get_camera_info(self, camera_frame_id):
        return self.static_tf_info_hub.get_camera_info(camera_frame_id)


#     def transform_init(self, data):
#         """通过所有参与的传感器之间的变换矩阵
#         - 通过 tf_info 和 camera_info 获取 lidar 到 image 的变换矩阵
#         - 通过 tf_info 获取 camera 到 lidar 的变换矩阵
#
#         Args:
#             data (_type_): _description_
#         """
#         pass
#
#     def calculate_lidar_to_image_transform(self, lidar_point_cloud):
#         pass
#
#     def calculate_camera_to_lidar_transform(self, camera_point_cloud):
#         pass
#
#     def visualize(self, data):
#         pass
