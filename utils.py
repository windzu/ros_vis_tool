import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R

# ros
import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import CameraInfo
from tf import transformations as t
from geometry_msgs.msg import Point


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

    for key, value in frame_id_info_dict.items():
        if frame_id_info_dict[key] is None:
            frame_id_info_dict[key] = []

    return frame_id_info_dict


def store_all_tf_info(frame_id_info_dict):
    lidar_frame_id_list = frame_id_info_dict["lidar_frame_id_list"]
    camera_frame_id_list = frame_id_info_dict["camera_frame_id_list"]
    radar_frame_id_list = frame_id_info_dict["radar_frame_id_list"]

    tf_info_dict = {}
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # lidar to camera
    for parent_frame_id in lidar_frame_id_list:
        if parent_frame_id not in tf_info_dict.keys():
            tf_info_dict[parent_frame_id] = {}

        for child_frame_id in camera_frame_id_list:
            tf_stamped = tf_buffer.lookup_transform(
                parent_frame_id, child_frame_id, rospy.Time.now(), rospy.Duration(1.0)
            )
            tf_info_dict[parent_frame_id][child_frame_id] = tf_stamped

    # lidar to radar
    for parent_frame_id in lidar_frame_id_list:
        if parent_frame_id not in tf_info_dict.keys():
            tf_info_dict[parent_frame_id] = {}
        for child_frame_id in radar_frame_id_list:
            tf_stamped = tf_buffer.lookup_transform(
                parent_frame_id, child_frame_id, rospy.Time.now(), rospy.Duration(1.0)
            )
            tf_info_dict[parent_frame_id][child_frame_id] = tf_stamped

    # camera to lidar
    for parent_frame_id in camera_frame_id_list:
        if parent_frame_id not in tf_info_dict.keys():
            tf_info_dict[parent_frame_id] = {}
        for child_frame_id in lidar_frame_id_list:
            tf_stamped = tf_buffer.lookup_transform(
                parent_frame_id, child_frame_id, rospy.Time.now(), rospy.Duration(1.0)
            )
            tf_info_dict[parent_frame_id][child_frame_id] = tf_stamped

    # camera to radar
    for parent_frame_id in camera_frame_id_list:
        if parent_frame_id not in tf_info_dict.keys():
            tf_info_dict[parent_frame_id] = {}
        for child_frame_id in radar_frame_id_list:
            tf_stamped = tf_buffer.lookup_transform(
                parent_frame_id, child_frame_id, rospy.Time.now(), rospy.Duration(1.0)
            )
            tf_info_dict[parent_frame_id][child_frame_id] = tf_stamped

    # radar to lidar
    for parent_frame_id in radar_frame_id_list:
        if parent_frame_id not in tf_info_dict.keys():
            tf_info_dict[parent_frame_id] = {}
        for child_frame_id in lidar_frame_id_list:
            tf_stamped = tf_buffer.lookup_transform(
                parent_frame_id, child_frame_id, rospy.Time.now(), rospy.Duration(1.0)
            )
            tf_info_dict[parent_frame_id][child_frame_id] = tf_stamped

    # radar to camera
    for parent_frame_id in radar_frame_id_list:
        if parent_frame_id not in tf_info_dict.keys():
            tf_info_dict[parent_frame_id] = {}
        for child_frame_id in camera_frame_id_list:
            tf_stamped = tf_buffer.lookup_transform(
                parent_frame_id, child_frame_id, rospy.Time.now(), rospy.Duration(1.0)
            )
            tf_info_dict[parent_frame_id][child_frame_id] = tf_stamped

    return tf_info_dict


def store_all_camera_info(camera_frame_id_list, camera_info_topic_suffix):
    camera_info_dict = {}
    for camera_frame_id in camera_frame_id_list:
        # debug
        print("[ store_all_camera_info ] : camera_frame_id: {}".format(camera_frame_id))
        camera_info_topic = camera_frame_id + camera_info_topic_suffix
        try:
            camera_info_msg = rospy.wait_for_message(camera_info_topic, CameraInfo, timeout=3.0)
            camera_info_dict[camera_frame_id] = camera_info_msg
        except rospy.ROSException:
            rospy.logwarn(
                "[ store_all_camera_info ] : timeout when waiting for camera_info_topic: {}".format(camera_info_topic)
            )
            continue
    return camera_info_dict


def calculate_transform_from_lidar_to_pixle(tf_stamped, camera_info):
    """calculate transform from lidar to pixel

    Args:
        tf_stamped (_type_): ros tf2_msgs/TFStamped msg lidar to camera
        camera_info (_type_): camera_info_msgs/CameraInfo msg camera info
    Returns:
        transform (np.array(3*4)): transform from lidar to pixel matrix
    """
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

    camera_info.P = np.array(camera_info.P).astype(np.float32)
    camera_projection_matrix = camera_info.P.reshape(3, 4)
    transform_matrix = np.dot(camera_projection_matrix, transform_matrix)
    return transform_matrix


def calculate_3d_bbox_corners(x, y, z, l, w, h, yaw):
    """将3d的bbox转换成8个corner 3d坐标 (右手坐标系,x朝前,y朝左,z朝上)

    Args:
        x (_type_): x position
        y (_type_): y position
        z (_type_): z position
        l (_type_): 3d bbox length (x-axis)
        w (_type_): 3d bbox width (y-axis)
        h (_type_): 3d bbox height (z-axis)



        yaw (_type_): rotation angle around the z-axis
    """

    # 1. get 8 3d points
    # 点的顺序为,以z=0将3d bbox分为上下两个部分 从顶部俯视角度看 以左上角为第一个点 按照顺时针顺序排列
    x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    y_corners = [h / 2, -h / 2, -h / 2, h / 2, h / 2, -h / 2, -h / 2, h / 2]
    z_corners = [w / 2, w / 2, w / 2, w / 2, -w / 2, -w / 2, -w / 2, -w / 2]
    # convert x_corners y_corners z_corners to 3x8 matrix
    corners_3d = np.array([x_corners, y_corners, z_corners])
    # roatate 4x8 matrix around z-axis by yaw angle
    r = R.from_euler("z", yaw)
    rotation_matrix = r.as_matrix()
    corners_3d = np.dot(rotation_matrix, corners_3d)
    # add x y z
    corners_3d = np.add(corners_3d, np.array([[x], [y], [z]]))
    # # expand 3x8 to 4x8 with 1s in the 4th column
    # corners_3d = np.concatenate((corners_3d, np.ones((1, 8))), axis=0)

    return corners_3d


def get_lines_from_8_points(points):
    """通过8个点获取n条线段

    Args:
        points (np.array(3*8)): 3d点投影到图像上的点的坐标
    """
    point_list = []
    # 0-3 line ”上平面“
    for i in range(4):
        start_point = Point()
        end_point = Point()
        if i == 3:
            start_point.x = points[0, i]
            start_point.y = points[1, i]
            start_point.z = 0
            end_point.x = points[0, 0]
            end_point.y = points[1, 0]
            end_point.z = 0
        else:
            start_point.x = points[0, i]
            start_point.y = points[1, i]
            start_point.z = 0
            end_point.x = points[0, i + 1]
            end_point.y = points[1, i + 1]
            end_point.z = 0
        point_list.append(start_point)
        point_list.append(end_point)
    # 4-7 line ”下平面“
    for i in range(4):
        start_point = Point()
        end_point = Point()
        i = i + 4
        if i == 7:
            start_point.x = points[0, i]
            start_point.y = points[1, i]
            start_point.z = 0
            end_point.x = points[0, 4]
            end_point.y = points[1, 4]
            end_point.z = 0
        else:
            start_point.x = points[0, i]
            start_point.y = points[1, i]
            start_point.z = 0
            end_point.x = points[0, i + 1]
            end_point.y = points[1, i + 1]
            end_point.z = 0
        point_list.append(start_point)
        point_list.append(end_point)
    # 8-11 line ”侧面“
    for i in range(4):
        start_point = Point()
        end_point = Point()
        start_point.x = points[0, i]
        start_point.y = points[1, i]
        start_point.z = 0
        end_point.x = points[0, i + 4]
        end_point.y = points[1, i + 4]
        end_point.z = 0
        point_list.append(start_point)
        point_list.append(end_point)
    return point_list
