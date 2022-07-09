import numpy as np
from scipy.spatial.transform import Rotation as R

# ros
import rospy
from visualization_msgs import ImageMarker


class LidarToImage:
    def __init__(self, tf_msg, camera_info):
        self.tf_msg = tf_msg
        self.camera_info = camera_info
        self.T = self.calculate_T(tf_msg, camera_info)

        print("[ LidarToImage ] : T = ", self.T)

    @staticmethod
    def calculate_T(tf_msg, camera_info):
        """计算lidar到image的变换矩阵"""
        parent_frame_id = tf_msg.header.frame_id
        child_frame_id = tf_msg.child_frame_id

        # camera_info frame_id should be the same as child_frame_id
        if child_frame_id != camera_info.header.frame_id:
            raise Exception("[ lidar_to_image ] : camera_info frame_id should be the same as child_frame_id")

        lidar_to_camera_T = tf_msg.transform
        rotation = R.from_quaternion(
            lidar_to_camera_T.rotation.x,
            lidar_to_camera_T.rotation.y,
            lidar_to_camera_T.rotation.z,
            lidar_to_camera_T.rotation.w,
        )
        rotation_matrix = rotation.as_matrix()
        translation = np.array(
            [
                lidar_to_camera_T.translation.x,
                lidar_to_camera_T.translation.y,
                lidar_to_camera_T.translation.z,
            ]
        )
        T = np.concatenate([rotation_matrix, translation[:, np.newaxis]], axis=1)
        T = np.dot(camera_info.P, T)
        return T

    def lidar_to_image(self, points):
        pass
