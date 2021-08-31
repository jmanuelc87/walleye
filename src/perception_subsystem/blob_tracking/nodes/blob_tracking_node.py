#! /usr/bin/env python
import rospy
from blob_tracking import BlobTracker
from blob_tracking.cfg import BlobTrackingConfig
from camera import CameraSensor
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import CompressedImage

# HSV limits default values
hsv_min = [0, 255, 255]
hsv_max = [23, 255, 255]

# We define the detection area [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
window = [0.1, 0.1, 0.9, 0.9]

ns = rospy.get_namespace()


def reconfigure_callback(config, level):
    rospy.loginfo("Reconfigure Request: %s", config)
    # Assign configuration values
    hsv_max[0] = config.h_max
    hsv_max[1] = config.s_max
    hsv_max[2] = config.v_max
    hsv_min[0] = config.h_min
    hsv_min[1] = config.s_min
    hsv_min[2] = config.v_min
    rospy.loginfo("%s, %s", hsv_max, hsv_min)
    return config


bridge = CvBridge()

pub_image = rospy.Publisher(ns + "camera/keypoints/compressed", CompressedImage, queue_size=1)

rospy.init_node("blob_tracking_node", log_level=rospy.DEBUG)

srv = Server(BlobTrackingConfig, reconfigure_callback)

blob_tracker = BlobTracker()
serviceImage = CameraSensor()

# node namespace
ns = rospy.get_namespace()

# define a loop rate
loop_rate = rospy.Rate(rospy.get_param(ns + "loop_rate"))

while not rospy.is_shutdown():
    # Get most recent image
    cv_image = serviceImage.get_image()

    # Detect blobs
    keypoints, _ = blob_tracker.blob_detect(cv_image, tuple(hsv_min), tuple(hsv_max), blur=3, blob_params=None,
                                            search_window=window)

    for keypoint in keypoints:
        x, y = blob_tracker.get_blob_relative_position(cv_image, keypoint)
        blob_size = keypoint.size
        blob_tracker.publish_blob(x, y, blob_size)

    image_with_keypoints = blob_tracker.draw_keypoints(cv_image, keypoints)

    try:
        pub_image.publish(bridge.cv2_to_compressed_imgmsg(image_with_keypoints, dst_format='jpeg'))
    except CvBridgeError as e:
        rospy.signal_shutdown("Error in CvBridge")

    loop_rate.sleep()

rospy.logwarn("Shutting down")
