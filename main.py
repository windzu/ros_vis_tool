import os
import sys
import argparse

# ros
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, CameraInfo, Image, CompressedImage

# local
from utils import parse_camera_info, parse_static_tf_info, parse_frame_id_info
from static_tf_info_hub import StaticTFInfoHub


def parse_args():
    parser = argparse.ArgumentParser(description="ros_vis_tool")
    parser.add_argument(
        "--frame_id_info_path", type=str, default="./config/frame_id_info.yaml", help="path to frame_id_info.yaml"
    )

    # topic info (for generate topic )
    parser.add_argument(
        "--camera_info_topic_suffix", type=str, default="/camera_info", help="used to generate camera info topic"
    )
    parser.add_argument(
        "--camera_topic_suffix", type=str, default="/image_rect_compressed", help="used to generate camera topic"
    )
    parser.add_argument("--lidar_topic_suffix", type=str, default="", help="used to generate lidar topic")

    # suffix
    args = parser.parse_args()
    return args


# def lidar_to_camera_callback():
#     # get tf from static_tf
#     static_tf_subscriber_dict = {}
#     for static_tf_frame_id in frame_id_info_dict["static_tf_frame_id_list"]:
#         static_tf_topic = static_tf_frame_id + args.static_tf_topic_suffix
#         static_tf_subscriber_dict[static_tf_topic] = rospy.Subscriber(static_tf_topic, TransformStamped, callback=None)
#
#     pass


def main():
    args = parse_args()

    rospy.init_node("ros_vis_tool")

    # topic info (for generate tf)
    frame_id_info_dict = parse_frame_id_info(args.frame_id_info_path)

    lidar_frame_id_list = frame_id_info_dict["lidar_frame_id_list"]
    camera_frame_id_list = frame_id_info_dict["camera_frame_id_list"]

    # 初始化用于查询 tf 的类(将所有需要使用到的tf信息都保存下来)
    static_tf_info_hub = StaticTFInfoHub(
        lidar_frame_id_list=lidar_frame_id_list,
        camera_frame_id_list=camera_frame_id_list,
        camera_info_topic_suffix=args.camera_info_topic_suffix,
    )

    # 接下来创建一个ros的可视化类
    # - 3d 点云 -> 2d 像素点
    # - 3d bbox -> 2d bbox

    # # test
    # camera_info = static_tf_info_hub.get_camera_info("CAM_FRONT")
    # print("test camera_info: ", camera_info)
    # trans = static_tf_info_hub.get_tf_info(parent_frame_id="LIDAR_TOP", child_frame_id="CAM_FRONT")
    # print("test tf_info: ", trans)
    # trans = static_tf_info_hub.get_tf_info(parent_frame_id="CAM_FRONT", child_frame_id="LIDAR_TOP")
    # print("test tf_info: ", trans)

    # 根据配置文件中的frame_id信息订阅所有与之相关的topic，包括
    # - camera_info : CameraInfo
    # - camera : Image or CompressedImage
    # - lidar : PointCloud2
    # - static_tf : TransformStamped (static_tf)

    # 首先获取到所有lidar到camera的tf信息，然后将其静态的存储起来，用于构建可视化类的初始化参数
    # - 所有lidar到所有camera的tf信息 ： 订阅lidar到camera的tf
    # - 所有camera的camera_info信息 ： 订阅camera的camera_info
    # - 所有camera到所有lidar的tf信息 ： 订阅camera到lidar的tf

    # subscribe all camera_info topic
    camera_info_subscriber_dict = {}
    sync_camera_subscriber_dict = {}
    for camera_frame_id in frame_id_info_dict["camera_frame_id_list"]:
        camera_info_topic = camera_frame_id + args.camera_info_topic_suffix
        camera_info_subscriber_dict[camera_info_topic] = rospy.Subscriber(camera_info_topic, CameraInfo, callback=None)

    # # subscribe all lidar topic
    # lidar_subscriber_dict = {}
    # sync_lidar_subscriber_dict = {}
    # for lidar_frame_id in frame_id_info_dict["lidar_frame_id_list"]:
    #     lidar_topic = lidar_frame_id + args.lidar_topic_suffix  # NOTE : lidar topic euqal to lidar frame id
    #     lidar_subscriber_dict[lidar_topic] = rospy.Subscriber(lidar_topic, PointCloud2, callback=None)
    #     sync_lidar_subscriber_dict[lidar_topic] = message_filters.Subscriber(lidar_topic, PointCloud2)

    # subscribe all static_tf topic between lidar and camera
    static_tf_subscriber_dict = {}
    for lidar_frame_id in frame_id_info_dict["lidar_frame_id_list"]:
        lidar_topic = lidar_frame_id + args.lidar_topic_suffix
        for camera_frame_id in frame_id_info_dict["camera_frame_id_list"]:
            camera_topic = camera_frame_id + args.camera_topic_suffix
        # static_tf_topic = static_tf_frame_id + args.static_tf_topic_suffix
    #   static_tf_subscriber_dict[static_tf_topic] = rospy.Subscriber(static_tf_topic, TransformStamped, callback=None)

    #     # 所有lidar到camera的投影
    #     lidar_to_camera_publisher_dict = {}  # 暂时使用
    #     for lidar_topic, sync_lidar_subscriber in sync_lidar_subscriber_dict.items():
    #         if len(sync_camera_info_subscriber_dict) != len(sync_camera_subscriber_dict):
    #             raise ValueError("camera_info_topic_suffix and camera_topic_suffix must be same length")
    #         for camera_frame_id in frame_id_info_dict["camera_frame_id_list"]:
    #             sync_camera_info_subscriber = sync_camera_info_subscriber_dict[
    #                 camera_frame_id + args.camera_info_topic_suffix
    #             ]
    #             sync_camera_subscriber = sync_camera_subscriber_dict[camera_frame_id + args.camera_topic_suffix]
    #
    #     ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)

    # subscribe all static_tf topic(目前只需要从lidar to camera的tf)
    static_tf_subscriber_dict = {}


#
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#
#         # project lidar point to camera
#         camera_frame_id_list = frame_id_info_dict["camera_frame_id_list"]
#         lidar_frame_id_list = frame_id_info_dict["lidar_frame_id_list"]
#
#         #
#
#         rate.sleep()


if __name__ == "__main__":
    main()
