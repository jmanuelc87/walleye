import os
import cv2
import rospy
import rospkg

import numpy as np

from jetcam import CSICamera
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def main():

    ns = rospy.get_namespace()

    use_csi_camera = rospy.get_param(ns + "use_csi_camera")

    width = rospy.get_param(ns + "width")

    height = rospy.get_param(ns + "height")

    rate = rospy.get_param(ns + "rate")

    npzfile = rospy.get_param(ns + "npzfilepath")

    usecalibration = rospy.get_param(ns + 'usecalibration')

    usecompressed = rospy.get_param(ns + 'usecompressed')

    rospack = rospkg.RosPack()

    rospy.init_node("camera_node", log_level=rospy.DEBUG)

    if use_csi_camera:
        camera = CSICamera(width=width, heigth=height)
    else:
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    loop = rospy.Rate(rate)

    image_pub = rospy.Publisher(ns + "image_raw", Image, queue_size=100)

    image_compressed_pub = rospy.Publisher(
        ns + "image_compressed", CompressedImage, queue_size=100)

    bridge = CvBridge()

    path = rospack.get_path('camera') + '/' + npzfile

    if usecalibration and os.path.exists(path):
        calibrationvalues = np.load(path)
        newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(
            calibrationvalues['mtx'], calibrationvalues['dist'], (width, height), 1, (width, height))

    while not rospy.is_shutdown():
        retval, image = camera.read()

        if usecalibration and newcameramatrix and calibrationvalues and roi:
            dst = cv2.undistort(
                image, calibrationvalues['mtx'], calibrationvalues['dist'], None, newcameramatrix)
            x, y, w, h = roi
            image = dst[y:y+h, x:x+w]

        if not retval:
            loop.sleep()
            continue

        try:
            if not usecompressed:
                image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='brg8'))

            if usecompressed:
                image_compressed_pub.publish(
                    bridge.cv2_to_compressed_imgmsg(image, dst_format='png'))
        except CvBridgeError as e:
            print(e)

        loop.sleep()


if __name__ == "__main__":
    main()
