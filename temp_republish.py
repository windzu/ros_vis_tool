# ros
import rospy
from sensor_msgs.msg import CompressedImage

rostopic = "/CAM_FRONT/image_rect_compressed"
repulish_topic = rostopic + "/republish"


rospy.init_node("temp_republish")


def callback(data):
    print("republish")
    data.header.stamp = rospy.Time.now()
    pub = rospy.Publisher(repulish_topic, CompressedImage, queue_size=10)
    pub.publish(data)


# subscriber
sub = rospy.Subscriber(rostopic, CompressedImage, callback)


rospy.spin()
