import os
import sys
import argparse

# ros
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import PointCloud2, CameraInfo, Image, CompressedImage

# local
from utils import parse_frame_id_info, store_all_camera_info, store_all_tf_info
from static_tf_info_hub import StaticTFInfoHub
from ros_visualizer import ROSVisualizer


def parse_args():
    parser = argparse.ArgumentParser(description="ros_vis_tool")
    parser.add_argument(
        "--frame_id_info_path", type=str, default="./config/frame_id_info.yaml", help="path to frame_id_info.yaml"
    )

    # if publish image_marker_from_lidar flag
    # lidar转pixel比较消耗性能，而且对延迟要求高，所以一般只在车辆静止时启用，用于校验lidar与camera的联合标定精度
    # TODO : 此参数当前的命名不太明确，后面再改
    parser.add_argument(
        "--if_publish_image_marker_from_lidar",
        type=bool,
        default=False,
        help="if publish image_marker_from_lidar",
    )

    # 默认情况下topic均由frame_id按照约定的规则生成，如果topic按照是按照约定的规则设置的，则下面的参数不需要设置
    # topic info (for generate topic )
    parser.add_argument("--lidar_topic_suffix", type=str, default="", help="lidar topic suffix")
    parser.add_argument("--camera_topic_suffix", type=str, default="/image_rect_compressed", help="camera topic suffix")
    parser.add_argument("--radar_topic_suffix", type=str, default="", help="radar topic suffix")
    parser.add_argument(
        "--camera_info_topic_suffix", type=str, default="/camera_info", help="used to generate camera info topic"
    )

    parser.add_argument("--republish_topic_suffix", type=str, default="/republish", help="msg republish topic suffix")
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    rospy.init_node("ros_vis_tool")

    # parse frame id info ,default have three kinds sensor (for generate tf)
    # - camera
    # - lidar
    # - radar
    frame_id_info_dict = parse_frame_id_info(args.frame_id_info_path)
    lidar_frame_id_list = frame_id_info_dict["lidar_frame_id_list"]
    camera_frame_id_list = frame_id_info_dict["camera_frame_id_list"]
    radar_frame_id_list = frame_id_info_dict["radar_frame_id_list"]

    # 提前保存所有tf_info (使用双重dict进行存储,第一个key为parent_frame_id,第二个key为child_frame_id)
    tf_info_dict = store_all_tf_info(frame_id_info_dict)
    # 提前保存所有camera_info
    camera_info_dict = store_all_camera_info(camera_frame_id_list, args.camera_info_topic_suffix)

    # debug
    print("camera_info_dict:", camera_info_dict)

    ros_visualizer = ROSVisualizer(
        lidar_frame_id_list=lidar_frame_id_list,
        lidar_topic_suffix=args.lidar_topic_suffix,
        camera_frame_id_list=camera_frame_id_list,
        camera_topic_suffix=args.camera_topic_suffix,
        radar_frame_id_list=radar_frame_id_list,
        radar_topic_suffix=args.radar_topic_suffix,
        tf_info_dict=tf_info_dict,
        camera_info_dict=camera_info_dict,
        if_publish_image_marker_from_lidar=args.if_publish_image_marker_from_lidar,  # 命名后面再改
    )

    # test
    ros_visualizer.start()


if __name__ == "__main__":
    main()
