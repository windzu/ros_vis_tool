import yaml
import numpy as np

# ros
import rospy
import geometry_msgs.msg
from sensor_msgs.msg import CameraInfo
from tf import transformations as t


class TFInfo:
    def __init__(self, tf_id, raw_tf_config=None):
        """存储tf的基本信息,包括四元数和平移向量。解析后格式为numpy.array的vector类型

        Args:
            tf_id (_type_): 两个frame的组合id:parent_frame_id_to_child_frame_id
            raw_tf_config (_type_): tf的原始信息
        """
        self.raw_tf_config = raw_tf_config
        # 基础信息
        self.tf_id = tf_id
        self.parent_frame_id = None
        self.child_frame_id = None
        self.rotation = None  # 四元数
        self.translation = None  # 平移向量

        self.serialize_tf_config()

    def serialize_tf_config(self):
        """从原始字典格式解析配置信息，包含四元数和平移向量"""

        def serialize_rotation(raw_tf_config):
            """解析旋转四元数元组"""
            if "rotation" in raw_tf_config.keys() and raw_tf_config["rotation"] is not None:
                rotation = raw_tf_config["rotation"]
                rotation = np.array(rotation, dtype=np.float32)
                rotation = rotation.reshape(
                    4,
                )
                return rotation
            else:
                raise Exception("[ tf_info ] : raw_tf_config中没有rotation")

        def serialize_translation(raw_tf_config):
            """解析平移向量"""
            if "translation" in raw_tf_config.keys() and raw_tf_config["translation"] is not None:
                translation = raw_tf_config["translation"]
                translation = np.array(translation, dtype=np.float32)
                translation = translation.reshape(
                    3,
                )
                return translation
            else:
                raise Exception("[ tf_info ] : raw_tf_config中没有translation")

        self.parent_frame_id = self.tf_id.split("_to_")[0]
        self.child_frame_id = self.tf_id.split("_to_")[1]

        if self.raw_tf_config is None:
            print("[ TFInfo ]  raw_tf_config is None")
            return
        self.rotation = serialize_rotation(self.raw_tf_config)
        self.translation = serialize_translation(self.raw_tf_config)

    def deserialize_tf_config(self):
        if self.raw_tf_config is None:
            self.raw_tf_config = dict()
        self.raw_tf_config["rotation"] = self.rotation.flatten().tolist()
        self.raw_tf_config["translation"] = self.translation.flatten().tolist()
        return self.raw_tf_config

    def __repr__(self) -> str:
        return "TFInfo(parent_frame_id: {}, child_frame_id: {}, rotation: {}, translation: {})".format(
            self.parent_frame_id,
            self.child_frame_id,
            self.rotation,
            self.translation,
        )


def parse_camera_info(camera_info_config_path):
    with open(camera_info_config_path, "r") as f:
        all_raw_tf_config = yaml.safe_load(f)

    camera_info_dict = {}
    for key, value in all_raw_tf_config.items():
        camera_frame_id = key
        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = camera_frame_id
        camera_info_msg.width = value["image_width"]
        camera_info_msg.height = value["image_height"]
        camera_info_msg.K = value["camera_matrix"]["data"]
        camera_info_msg.D = value["distortion_coefficients"]["data"]
        camera_info_msg.R = value["rectification_matrix"]["data"]
        camera_info_msg.P = value["projection_matrix"]["data"]
        camera_info_msg.distortion_model = value["distortion_model"]
        camera_info_dict[key] = camera_info_msg
    return camera_info_dict


def parse_static_tf_info(tf_config_path):
    with open(tf_config_path, "r") as f:
        all_raw_tf_config = yaml.safe_load(f)

    static_transformStamped_list = []
    for key, value in all_raw_tf_config.items():
        tf_info = TFInfo(key, value)
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = tf_info.parent_frame_id
        static_transformStamped.child_frame_id = tf_info.child_frame_id
        static_transformStamped.transform.translation.x = float(tf_info.translation[0])
        static_transformStamped.transform.translation.y = float(tf_info.translation[1])
        static_transformStamped.transform.translation.z = float(tf_info.translation[2])

        static_transformStamped.transform.rotation.x = float(tf_info.rotation[0])
        static_transformStamped.transform.rotation.y = float(tf_info.rotation[1])
        static_transformStamped.transform.rotation.z = float(tf_info.rotation[2])
        static_transformStamped.transform.rotation.w = float(tf_info.rotation[3])
        static_transformStamped_list.append(static_transformStamped)
    return static_transformStamped_list


def build_tf_lookup_table(tf_config_path):
    """构建一个所有frame_id之间的tf的映射表"""
    with open(tf_config_path, "r") as f:
        all_raw_tf_config = yaml.safe_load(f)
    tf_lookup_table = {}
    for key, value in all_raw_tf_config.items():
        tf_info = TFInfo(key, value)
        rotation_matrix = t.quaternion_matrix(tf_info.rotation)
        translation_matrix = t.translation_matrix(tf_info.translation)
        transform = rotation_matrix
        transform[:3, 3] = translation_matrix[:3, 3]
        inverse_transform = t.inverse_matrix(transform)
        tf_lookup_table[tf_info.parent_frame_id + "_to_" + tf_info.child_frame_id] = transform
        tf_lookup_table[tf_info.child_frame_id + "_to_" + tf_info.parent_frame_id] = inverse_transform
    return tf_lookup_table


# def build_lidar_to_pixel_lookup_table(tf_config_path):


def parse_frame_id_info(frame_id_info_path):
    with open(frame_id_info_path, "r") as f:
        all_raw_config = yaml.safe_load(f)

    frame_id_info_dict = all_raw_config

    return frame_id_info_dict
