from logging import error
import time
import rospy
import tf2_ros
from sensor_msgs.msg import CameraInfo


class StaticTFInfoHub:
    def __init__(self, lidar_frame_id_list, camera_frame_id_list, camera_info_topic_suffix="/camera_info"):
        self.lidar_frame_id_list = lidar_frame_id_list
        self.camera_frame_id_list = camera_frame_id_list
        self.camera_info_topic_suffix = camera_info_topic_suffix

        # protected变量
        self._camera_info_dict = {}
        ##  tf info
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # debug
        print("start _camera_info_init")

        self._camera_info_init()

    def get_camera_info(self, camera_frame_id):
        """根据相机的frame_id获取相机的camera_info信息

        Args:
            camera_frame_id (str): 相机的frame_id
        """
        if camera_frame_id in self._camera_info_dict:
            return self._camera_info_dict[camera_frame_id]
        else:
            raise Exception("camera_frame_id is not in camera_info_dict")

    def get_tf_info(self, parent_frame_id, child_frame_id):
        try:
            start_time = time.time()
            trans = self._tf_buffer.lookup_transform(parent_frame_id, child_frame_id, rospy.Time())
            end_time = time.time()
            print("listern tf info cost time : ", end_time - start_time)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise Exception("get_tf_info error")
        return trans

    def _camera_info_init(self):
        # subscribe all camera_info topic
        for camera_frame_id in self.camera_frame_id_list:
            # debug
            print("camera_frame_id is :", camera_frame_id)
            camera_info_topic = camera_frame_id + self.camera_info_topic_suffix
            camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo, timeout=None)
            self._camera_info_dict[camera_frame_id] = camera_info

            if camera_frame_id in self._camera_info_dict:
                if isinstance(self._camera_info_dict[camera_frame_id], CameraInfo):
                    pass
            else:
                raise Exception("camera_frame_id is not in camera_info_dict")
