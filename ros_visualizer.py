from copy import deepcopy
from turtle import color, distance
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import copy
import math


# ros
import rospy
import tf2_ros
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from geometry_msgs.msg import Point


# from visualization_msgs import ImageMarker
from visualization_msgs.msg import ImageMarker, Marker, MarkerArray
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from foxglove_msgs.msg import ImageMarkerArray

# pypcd
from pypcd import pypcd

# local
from utils import calculate_3d_bbox_corners, calculate_transform_from_lidar_to_pixle, get_lines_from_8_points

color_dict = {
    "red": ColorRGBA(1.0, 0.0, 0.0, 0.8),
    "green": ColorRGBA(0.0, 1.0, 0.0, 0.8),
    "blue": ColorRGBA(0.0, 0.0, 1.0, 0.8),
    "yellow": ColorRGBA(1.0, 1.0, 0.0, 0.8),
    "white": ColorRGBA(1.0, 1.0, 1.0, 0.8),
    "black": ColorRGBA(0.0, 0.0, 0.0, 0.8),
    "purple": ColorRGBA(1.0, 0.0, 1.0, 0.8),
    "orange": ColorRGBA(1.0, 0.5, 0.0, 0.8),
}


class ROSVisualizer:
    def __init__(
        self,
        lidar_frame_id_list=None,
        lidar_topic_suffix="",
        camera_frame_id_list=None,
        camera_topic_suffix="",
        radar_frame_id_list=None,
        radar_topic_suffix="",
        tf_info_dict=None,
        camera_info_dict=None,
        if_publish_image_marker_from_lidar=False,
    ):
        self.lidar_frame_id_list = lidar_frame_id_list
        self.lidar_topic_suffix = lidar_topic_suffix
        self.camera_frame_id_list = camera_frame_id_list
        self.camera_topic_suffix = camera_topic_suffix
        self.radar_frame_id_list = radar_frame_id_list
        self.radar_topic_suffix = radar_topic_suffix
        self.tf_info_dict = tf_info_dict
        self.camera_info_dict = camera_info_dict
        self.if_publish_image_marker_from_lidar = if_publish_image_marker_from_lidar

        # global variable
        self.lidar_to_pixle_transform_matrix_dict = {}

        # 初始化所有订阅者
        self.init_all_subscriber()
        # 初始化所有发布者
        self.init_all_publisher()
        # 初始化所有transform
        self.init_all_transform()

        # color dict
        self.color_dict = {
            "red": ColorRGBA(1.0, 0.0, 0.0, 0.8),
            "green": ColorRGBA(0.0, 1.0, 0.0, 0.8),
            "blue": ColorRGBA(0.0, 0.0, 1.0, 0.8),
            "yellow": ColorRGBA(1.0, 1.0, 0.0, 0.8),
            "cyan": ColorRGBA(0.0, 1.0, 1.0, 0.8),
            "magenta": ColorRGBA(1.0, 0.0, 1.0, 0.8),
            "white": ColorRGBA(1.0, 1.0, 1.0, 0.8),
            "black": ColorRGBA(0.0, 0.0, 0.0, 0.8),
        }

    def init_all_subscriber(self):
        # lidar
        self.lidar_subscriber_dict = self.__init_all_lidar_subsciber()
        self.lidar_msg_dict = {}
        # camera
        self.camera_subscriber_dict = self.__init_all_camera_subsciber()
        self.camera_msg_dict = {}
        # radar
        self.radar_subscriber_dict = self.__init_all_radar_subsciber()
        # detected_objects
        self.detected_objects_subscriber_dict = self.__init_all_detected_objects_subscriber()
        self.detected_objects_msg_dict = {}

    def init_all_publisher(self):

        # image_marker_from_lidar
        self.image_marker_from_lidar_publisher_dict = self.__init_all_image_marker_from_lidar_publisher()
        # image_marker_array_from_detected_objects
        self.image_marker_array_from_detected_objects_publisher_dict = (
            self.__init_all_image_marker_array_from_detected_objects_publisher()
        )

        # marker_array from detected_objects
        self.marker_array_from_detected_objects_publisher_dict = (
            self.__init_all_marker_array_from_detected_objects_publisher()
        )

    def init_all_transform(self):
        # 已经有了所有frame_id之间的变换信息，还缺少如下一些变换信息
        # lidar -> camera pixel
        for lidar_frame_id in self.lidar_frame_id_list:
            temp_dict = {}
            for camera_frame_id in self.camera_frame_id_list:
                tf_stamped = self.tf_info_dict[lidar_frame_id][camera_frame_id]
                camera_info = self.camera_info_dict[camera_frame_id]
                transform_matrix = calculate_transform_from_lidar_to_pixle(tf_stamped, camera_info)
                temp_dict[camera_frame_id] = transform_matrix
            self.lidar_to_pixle_transform_matrix_dict[lidar_frame_id] = temp_dict

    def __init_all_lidar_subsciber(self):
        """init all lidar subscriber which lidar topic based on lidar_frame_id_list

        Returns:
            dict: all lidar topics's subscriber
        """
        if self.lidar_frame_id_list is None:
            return {}

        lidar_subscriber_dict = {}
        for lidar_frame_id in self.lidar_frame_id_list:
            lidar_topic = lidar_frame_id + self.lidar_topic_suffix
            lidar_subscriber_dict[lidar_frame_id] = rospy.Subscriber(lidar_topic, PointCloud2, self.__lidar_callback)
        return lidar_subscriber_dict

    def __init_all_camera_subsciber(self):
        """init all camera subscriber which camera topic based on camera_frame_id_list

        Returns:
            dict: all camera topics's subscriber
        """
        if self.camera_info_dict is None:
            return {}

        camera_subscriber_dict = {}
        for camera_frame_id in self.camera_frame_id_list:
            camera_topic = camera_frame_id + self.camera_topic_suffix
            camera_subscriber_dict[camera_frame_id] = rospy.Subscriber(
                camera_topic, CompressedImage, self.__camera_callback
            )
        return camera_subscriber_dict

    def __init_all_radar_subsciber(self):
        """init all radar subscriber which radar topic based on radar_frame_id_list

        Returns:
            dict: all radar topics's subscriber
        """
        if self.radar_frame_id_list is None:
            return {}

        radar_subscriber_dict = {}
        for radar_frame_id in self.radar_frame_id_list:
            radar_topic = radar_frame_id + self.radar_topic_suffix
            radar_subscriber_dict[radar_frame_id] = rospy.Subscriber(radar_topic, PointCloud2, self.__radar_callback)
        return radar_subscriber_dict

    def __init_all_detected_objects_subscriber(self):
        """init all detected objects subscriber which detected objects topic based on lidar_frame_id_list and camera_frame_id_list

        Returns:
            dict: all detected objects topics's subscriber
        """
        detected_objects_subscriber_dict = {}
        # lidar
        if self.lidar_frame_id_list is not None:
            for lidar_frame_id in self.lidar_frame_id_list:
                detected_objects_topic = lidar_frame_id + "/detected_objects"
                detected_objects_subscriber_dict[lidar_frame_id] = rospy.Subscriber(
                    detected_objects_topic, DetectedObjectArray, self.__detected_objects_callback
                )
        # camera
        if self.camera_info_dict is not None:
            for camera_frame_id in self.camera_frame_id_list:
                detected_objects_topic = camera_frame_id + "/detected_objects"
                detected_objects_subscriber_dict[camera_frame_id] = rospy.Subscriber(
                    detected_objects_topic, DetectedObjectArray, self.__detected_objects_callback
                )
        # radar
        if self.radar_frame_id_list is not None:
            for radar_frame_id in self.radar_frame_id_list:
                detected_objects_topic = radar_frame_id + "/detected_objects"
                detected_objects_subscriber_dict[radar_frame_id] = rospy.Subscriber(
                    detected_objects_topic, DetectedObjectArray, self.__detected_objects_callback
                )
        return detected_objects_subscriber_dict

    def __init_all_image_marker_from_lidar_publisher(self):
        """init all image marker from lidar which image topic based on camera_frame_id_list
        Returns:
            dict: all image marker from lidar's publisher
        """
        image_marker_from_lidar_publisher_dict = {}
        for camera_frame_id in self.camera_frame_id_list:
            image_marker_topic = camera_frame_id + "image_marker_from_lidar"
            image_marker_from_lidar_publisher_dict[camera_frame_id] = rospy.Publisher(
                image_marker_topic, ImageMarker, queue_size=10
            )
        return image_marker_from_lidar_publisher_dict

    def __init_all_marker_array_from_detected_objects_publisher(self):
        """init all marker array from detected objects which marker array topic based on lidar_frame_id_list
        Returns:
            dict: all marker array from detected objects's publisher
        """
        marker_array_from_detected_objects_publisher_dict = {}
        for lidar_frame_id in self.lidar_frame_id_list:
            marker_array_topic = lidar_frame_id + "/marker_array_from_detected_objects"
            marker_array_from_detected_objects_publisher_dict[lidar_frame_id] = rospy.Publisher(
                marker_array_topic, MarkerArray, queue_size=10
            )
        return marker_array_from_detected_objects_publisher_dict

    def __init_all_image_marker_array_from_detected_objects_publisher(self):
        """init all image marker array from detected objects which image topic based on camera_frame_id_list
        Returns:
            dict: all image marker array from detected objects's publisher
        """
        image_marker_array_from_detected_objects_publisher_dict = {}
        for camera_frame_id in self.camera_frame_id_list:
            image_marker_array_topic = camera_frame_id + "/image_marker_array_from_detected_objects"
            image_marker_array_from_detected_objects_publisher_dict[camera_frame_id] = rospy.Publisher(
                image_marker_array_topic, ImageMarkerArray, queue_size=10
            )
        return image_marker_array_from_detected_objects_publisher_dict

    def __lidar_callback(self, pointcloud2):
        lidar_frame_id = pointcloud2.header.frame_id
        self.lidar_msg_dict[lidar_frame_id] = pointcloud2

        # 接收到的点云数据，转换为在图像上的投影marker，并发布出去
        if self.if_publish_image_marker_from_lidar:
            for camera_frame_id in self.camera_frame_id_list:
                lidar_to_pixle_transform_matrix = self.lidar_to_pixle_transform_matrix_dict[lidar_frame_id][
                    camera_frame_id
                ]
                camera_info = self.camera_info_dict[camera_frame_id]

                # timing
                start_time = time.time()
                image_marker_from_lidar = ROSVisualizer.pointcloud_to_pixel(
                    lidar_to_pixle_transform_matrix=lidar_to_pixle_transform_matrix,
                    camera_info=camera_info,
                    pointcloud2=pointcloud2,
                )
                end_time = time.time()
                print("pointcloud to pixel cost time : ", end_time - start_time)
                self.image_marker_from_lidar_publisher_dict[camera_frame_id].publish(image_marker_from_lidar)

    def __camera_callback(self, image_msg):
        camera_frame_id = image_msg.header.frame_id
        self.camera_msg_dict[camera_frame_id] = image_msg

    def __radar_callback(self, pointcloud2):
        radar_frame_id = pointcloud2.header.frame_id
        self.radar_msg_dict[radar_frame_id] = pointcloud2

        # 接收到的点云数据，转换为在图像上的投影marker，并发布出去
        if self.if_publish_image_marker_from_radar:
            for camera_frame_id in self.camera_frame_id_list:
                lidar_to_pixle_transform_matrix = self.lidar_to_pixle_transform_matrix_dict[radar_frame_id][
                    camera_frame_id
                ]
                camera_info = self.camera_info_dict[camera_frame_id]
                # timing
                start_time = time.time()
                image_marker_from_radar = ROSVisualizer.pointcloud_to_pixel(
                    lidar_to_pixle_transform_matrix=lidar_to_pixle_transform_matrix,
                    camera_info=camera_info,
                    pointcloud2=pointcloud2,
                )
                end_time = time.time()
                print("pointcloud to pixel cost time : ", end_time - start_time)
                self.image_marker_from_radar_publisher_dict[camera_frame_id].publish(image_marker_from_radar)

    def __detected_objects_callback(self, detected_object_array):
        if not detected_object_array.objects:
            return

        detected_objects_frame_id = detected_object_array.objects[0].header.frame_id
        self.detected_objects_msg_dict[detected_objects_frame_id] = detected_object_array

        # 接收到的检测结果，转换为在图像上的投影marker和在lidar上的投影marker，并发布出去
        # - 如果是3d检测结果，则发布在图像上的投影marker和在lidar上的投影marker
        # - 如果是2d检测结果，则发布在图像上的投影marker

        # test
        detected_objects_type = "3d"
        if detected_objects_type == "3d":
            # convert detected objects to marker array
            lidar_frame_id = detected_object_array.objects[0].header.frame_id
            marker_array_from_detected_objects = ROSVisualizer.autoware_detected_object_array_to_marker_array(
                detected_object_array
            )

            self.marker_array_from_detected_objects_publisher_dict[lidar_frame_id].publish(
                marker_array_from_detected_objects
            )
            # convert detected objects to image marker array
            # NOTE : will get a image_marker_array dict, and the key is the camera_frame_id
            image_marker_array_from_detected_objects_dict = (
                ROSVisualizer.autoware_detected_object_array_to_image_marker_array(
                    detected_object_array=detected_object_array,
                    lidar_to_pixle_transform_matrix_dict=self.lidar_to_pixle_transform_matrix_dict,
                )
            )
            for camera_frame_id in self.camera_frame_id_list:
                image_marker_array_from_detected_objects_publisher_dict = (
                    self.image_marker_array_from_detected_objects_publisher_dict[camera_frame_id]
                )
                image_marker_array_from_detected_objects_publisher_dict.publish(
                    image_marker_array_from_detected_objects_dict[camera_frame_id]
                )

    def start(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # all lidar to camera
            # pass
            # ros log info
            rospy.loginfo("ros_visualizer is running")
            rate.sleep()

    @staticmethod
    def autoware_detected_object_array_to_marker_array(detected_object_array):
        if detected_object_array is None or not isinstance(detected_object_array, DetectedObjectArray):
            return None

        def create_cube_marker(object, id):
            bbox3d_marker = Marker()
            bbox3d_marker.header = object.header
            bbox3d_marker.header.stamp = rospy.Time.now()
            bbox3d_marker.type = Marker.CUBE
            bbox3d_marker.action = Marker.ADD
            bbox3d_marker.ns = "CUBE"
            bbox3d_marker.id = id

            bbox3d_marker.pose = object.pose
            bbox3d_marker.scale = object.dimensions
            bbox3d_marker.color = ROSVisualizer.bbox_color_map(
                bbox_type="detected", bbox_class=object.label
            )  # need to change
            bbox3d_marker.lifetime = rospy.Duration(0.1)
            return bbox3d_marker

        def create_arrow_marker(object, id):
            bbox3d_marker = Marker()
            bbox3d_marker.header = object.header
            bbox3d_marker.header.stamp = rospy.Time.now()
            bbox3d_marker.type = Marker.ARROW
            bbox3d_marker.action = Marker.ADD
            bbox3d_marker.ns = "ARROW"
            bbox3d_marker.id = id

            bbox3d_marker.pose = object.pose
            bbox3d_marker.scale.x = object.dimensions.x
            bbox3d_marker.scale.y = 1
            bbox3d_marker.scale.z = 1
            bbox3d_marker.color = color_dict["red"]
            bbox3d_marker.lifetime = rospy.Duration(0.1)
            return bbox3d_marker

        marker_array = MarkerArray()
        id = 0
        for object in detected_object_array.objects:
            # cube marker
            cube_marker = create_cube_marker(object, id)
            marker_array.markers.append(cube_marker)
            id += 1
            # arrow marker
            arrow_marker = create_arrow_marker(object, id)
            marker_array.markers.append(arrow_marker)
            id += 1
        return marker_array

    @staticmethod
    def autoware_detected_object_array_to_image_marker_array(
        detected_object_array, lidar_to_pixle_transform_matrix_dict
    ):
        """将3d检测结果转换为在图像上的投影marker
        NOTE : 一个3d检测结果可能会出现在多个相机上,所以返回的是一个字典,key是相机的frame_id

        Args:
            detected_object_array (_type_): _description_
            lidar_to_pixle_transform_matrix_dict (_type_): each lidar to each camera's pixel transform matrix

        Returns:
            dict: image_marker_array dict , the key is the camera_frame_id
        """
        if detected_object_array is None or not isinstance(detected_object_array, DetectedObjectArray):
            return {}

        image_marker_array_dict = {}
        # first get the all transform from lidar to camera pixel
        current_lidar_frame_id = detected_object_array.objects[0].header.frame_id
        current_lidar_to_pixle_transform_matrix_dict = lidar_to_pixle_transform_matrix_dict[current_lidar_frame_id]

        # NOTE : foxglove ImageMarkerArray just need one image_marker
        image_marker_array = ImageMarkerArray()
        image_marker = ImageMarker()
        image_marker.id = 0
        image_marker.ns = "autoware_detected_object"  #  autoware_detected_object
        image_marker.header = detected_object_array.objects[0].header
        image_marker.header.stamp = rospy.Time.now()
        image_marker.type = ImageMarker.LINE_LIST
        image_marker.action = ImageMarker.ADD
        image_marker.scale = 2
        image_marker.points = []
        image_marker.outline_colors = []
        image_marker_array.markers.append(image_marker)

        for key, value in current_lidar_to_pixle_transform_matrix_dict.items():
            camera_frame_id = key
            image_marker_array_dict[camera_frame_id] = copy.deepcopy(image_marker_array)
            image_marker_array_dict[camera_frame_id].markers[0].header.frame_id = camera_frame_id

        for object in detected_object_array.objects:
            # get h, w, l, x, y, z, yaw, trans by parsing object
            x = object.pose.position.x
            y = object.pose.position.y
            z = object.pose.position.z
            l = object.dimensions.x
            w = object.dimensions.y
            h = object.dimensions.z

            # get yaw
            r = R.from_quat(
                [
                    object.pose.orientation.x,
                    object.pose.orientation.y,
                    object.pose.orientation.z,
                    object.pose.orientation.w,
                ]
            )
            yaw = r.as_euler("XYZ")[2]

            # debug
            yaw = -yaw  # 反方向

            #             # debug
            #             print("x, y, z : ", x, y, z)
            #             print("l, w, h : ", l, w, h)
            #             print("yaw : ", yaw)
            #             # debug
            #             # yaw equal yaw - pi/2
            #
            #             # yaw = math.pi / 2 - yaw  # 0

            corners_3d = calculate_3d_bbox_corners(x, y, z, l, w, h, yaw)
            # expand 3x8 to 4x8 with 1s in the 4th column
            corners_3d = np.concatenate((corners_3d, np.ones((1, 8))), axis=0)

            # lidar to each camera pixel transform
            for key, value in current_lidar_to_pixle_transform_matrix_dict.items():
                camera_frame_id = key
                lidar_to_pixel_transform_matrix = value
                points = np.dot(lidar_to_pixel_transform_matrix, corners_3d)
                # post process
                points = (points / points[2, :]).astype(np.float32)

                point_list = get_lines_from_8_points(points)
                image_marker_array_dict[camera_frame_id].markers[0].points.extend(point_list)
                # TODO : add color
                # get 8 points' color

                red_color = color_dict["red"]
                color_list = []
                for i in range(12):
                    color_list.append(red_color)
                image_marker_array_dict[camera_frame_id].markers[0].outline_colors.extend(color_list)

        return image_marker_array_dict

    @staticmethod
    def pointcloud_to_pixel(lidar_to_pixle_transform_matrix, camera_info, pointcloud2):
        """convert pointcloud2 to image marker
        args:
            tf_stamped: ros tf_stamped msg
            camera_info: ros camera_info msg
            pointcloud2: pointcloud2 msg
        return:
            image_marker: image_marker

        """

        # timing test
        new_start_time = time.time()

        pc = pypcd.PointCloud.from_msg(pointcloud2)
        x = pc.pc_data["x"].flatten()
        y = pc.pc_data["y"].flatten()
        z = pc.pc_data["z"].flatten()
        points = np.zeros((4, x.shape[0]))
        points[0, :] = x
        points[1, :] = y
        points[2, :] = z
        points[3, :] = 1.0

        points = np.dot(lidar_to_pixle_transform_matrix, points)

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

        new_end_time = time.time()
        print("parse pointcloud and convert cost time :", new_end_time - new_start_time)

        # create image marker
        image_marker = ImageMarker()
        image_marker.header.frame_id = camera_info.header.frame_id
        image_marker.header.stamp = rospy.Time.now()
        image_marker.ns = pointcloud2.header.frame_id
        image_marker.id = 0
        image_marker.type = ImageMarker.POINTS
        image_marker.action = image_marker.ADD
        image_marker.scale = 2
        image_marker.lifetime = rospy.Duration(0.0)

        # TODO : reduce create Point time
        for i in range(points.shape[1]):
            point = Point()
            point.x = points[0, i]
            point.y = points[1, i]
            point.z = 0.0
            image_marker.points.append(point)

            color = ROSVisualizer.distance_color_map(cur_depth=distance[i])
            image_marker.outline_colors.append(color)

        return image_marker

    @staticmethod
    def bbox_color_map(bbox_type="detected", bbox_class="none"):
        """return color for bbox
        args:
            bbox_type: bbox type (eg. detected, predicted))
            bbox_class: bbox class (eg. car, pedestrian, cyclist)
        return:
            color: color

        """
        # NOTE : 以下将常见的类别作为颜色映射
        # (紫色)包含人 ： 人 行人 自行车 摩托车
        # (黄色)小汽车 ： 汽车
        # (橙色)大汽车 ： 卡车 公交车 拖车 建筑车
        # (红色)静态障碍物 ： 锥形桶 栅栏

        bbox_class = bbox_class.lower()
        if bbox_type == "detected":
            if bbox_class in ["pedestrian", "person", "bicycle", "motorcycle", "cyclist"]:
                color = color_dict["purple"]
            elif bbox_class in ["car"]:
                color = color_dict["yellow"]
            elif bbox_class in ["truck", "bus", "trailer", "construction_vehicle"]:
                color = color_dict["orange"]
            elif bbox_class in ["traffic_cone", "barrier"]:
                color = color_dict["red"]
            elif bbox_class == "other":
                color = color_dict["blue"]
            else:
                color = color_dict["white"]
        elif bbox_type == "predicted":
            color = color_dict["white"]
        else:
            color = color_dict["white"]
        return color

    @staticmethod
    def distance_color_map(cur_depth, max_depth=70, min_depth=0):
        """根据输入的深度值，计算一个颜色值

        Args:
            cur_depth (_type_): 当前深度值
            max_depth (int, optional): _description_. Defaults to 70.
            min_depth (int, optional): _description_. Defaults to 0.

        Returns:
            ColorRGBA: 颜色值
        """
        color_map_scale = (max_depth - min_depth) / 10
        color = ColorRGBA()
        if cur_depth < min_depth:
            color.r = 0.0
            color.g = 0.0
            color.b = 1
            color.a = 1
        elif cur_depth < min_depth + color_map_scale:
            color.r = 0.0
            color.g = (cur_depth - min_depth) / color_map_scale
            color.b = 1
            color.a = 1
        elif cur_depth < min_depth + color_map_scale * 2:
            color.r = 0.0
            color.g = 1
            color.b = 1 - (cur_depth - min_depth - color_map_scale) / color_map_scale
            color.a = 1
        elif cur_depth < min_depth + color_map_scale * 3:
            color.r = (cur_depth - min_depth - color_map_scale * 2) / color_map_scale
            color.g = 1
            color.b = 0.0
            color.a = 1
        elif cur_depth < min_depth + color_map_scale * 4:
            color.r = 1
            color.g = 1 - (cur_depth - min_depth - color_map_scale * 3) / color_map_scale
            color.b = 0.0
            color.a = 1
        elif cur_depth < min_depth + color_map_scale * 5:
            color.r = 1
            color.g = 0.0
            color.b = 0.0
            color.a = 1
        elif cur_depth < min_depth + color_map_scale * 6:
            color.r = 1 - (cur_depth - min_depth - color_map_scale * 5) / color_map_scale
            color.g = 0.0
            color.b = 0.0
            color.a = 1
        elif cur_depth < min_depth + color_map_scale * 7:
            color.r = 0.0
            color.g = 0.0
            color.b = 1 - (cur_depth - min_depth - color_map_scale * 6) / color_map_scale
            color.a = 1
        elif cur_depth < min_depth + color_map_scale * 8:
            color.r = 0.0
            color.g = 1 - (cur_depth - min_depth - color_map_scale * 7) / color_map_scale
            color.b = 1
            color.a = 1
        elif cur_depth < min_depth + color_map_scale * 9:
            color.r = 1 - (cur_depth - min_depth - color_map_scale * 8) / color_map_scale
            color.g = 1 - (cur_depth - min_depth - color_map_scale * 8) / color_map_scale
            color.b = 1
            color.a = 1
        else:
            color.r = 0.0
            color.g = 1
            color.b = 1
            color.a = 1
        return color
