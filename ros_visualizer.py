# ros
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs import ImageMarker

# local
from utils import parse_frame_id_info


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

    def lidar_to_camera(self, lidar_frame_id, camera_frame_id, covert_mode):
        tf_info = self.get_tf_info(lidar_frame_id, camera_frame_id)
        camera_info = self.get_camera_info(camera_frame_id)

        if covert_mode == "3DPOINTS":
            lidar_topic = lidar_frame_id + self.lidar_topic_suffix
            pointcloud = rospy.wait_for_message(lidar_topic, PointCloud2, timeout=None)
            self._convert_3dpoints_to_pixel(tf_info=tf_info, camera_info=camera_info, pointcloud=pointcloud)
        elif covert_mode == "3DBBOX":
            pass

    @staticmethod
    def _convert_3dpoints_to_pixel(tf_info, camera_info, pointcloud):
        image_marker = ImageMarker()

    def get_tf_info(self, parent_frame_id, child_frame_id):
        return self.static_tf_info_hub(parent_frame_id=parent_frame_id, child_frame_id=child_frame_id)

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
