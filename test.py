import argparse

# ros
import rospy
import tf2_ros
from sensor_msgs.msg import CameraInfo

# local
from utils import parse_camera_info, parse_static_tf_info, parse_frame_id_info


def parse_args():
    parser = argparse.ArgumentParser(description="ros_vis_tool")
    parser.add_argument(
        "--camera_info_path", type=str, default="./config/camera_info.yaml", help="path to camera_info.yaml"
    )
    parser.add_argument(
        "--static_tf_info_path", type=str, default="./config/static_tf_info.yaml", help="path to static_tf_info.yaml"
    )
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    rospy.init_node("tf_info_and_camera_info_node")

    # camera info
    # NOTE : camera topic euqal to camera frame id + "/camera_info"
    camera_info_dict = parse_camera_info(args.camera_info_path)
    camera_info_publisher_dict = {
        key + "/camera_info": rospy.Publisher(key + "/camera_info", CameraInfo, queue_size=10)
        for key in camera_info_dict.keys()
    }

    # static tf info
    static_transformStamped_list = parse_static_tf_info(args.static_tf_info_path)
    static_tf2_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf2_broadcaster.sendTransform(static_transformStamped_list)

    rate = rospy.Rate(10)
    # publish all camera info
    while not rospy.is_shutdown():
        for key, value in camera_info_dict.items():
            camera_info_publisher_dict[key + "/camera_info"].publish(value)

        print("publishing...")

        rate.sleep()


if __name__ == "__main__":
    main()
