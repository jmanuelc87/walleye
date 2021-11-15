#! /usr/bin/env python3
import cv2
import numpy as np
import rospy
from blob_tracking import BlobTracker
from camera import CameraSensor, rescaleFrame


def nothing(x):
    pass


# Create a window
cv2.namedWindow('Blob HSV Calibrator')

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('Hue Min', 'Blob HSV Calibrator', 0, 179, nothing)
cv2.createTrackbar('Saturation Min', 'Blob HSV Calibrator', 0, 255, nothing)
cv2.createTrackbar('Value Min', 'Blob HSV Calibrator', 0, 255, nothing)
cv2.createTrackbar('Hue Max', 'Blob HSV Calibrator', 0, 179, nothing)
cv2.createTrackbar('Saturation Max', 'Blob HSV Calibrator', 0, 255, nothing)
cv2.createTrackbar('Value Max', 'Blob HSV Calibrator', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('Hue Max', 'Blob HSV Calibrator', 179)
cv2.setTrackbarPos('Saturation Max', 'Blob HSV Calibrator', 255)
cv2.setTrackbarPos('Value Max', 'Blob HSV Calibrator', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

rospy.init_node('blob_hsv_calibrator_node', log_level=rospy.DEBUG)

camera_service = CameraSensor(compressed=True)
blob_tracker = BlobTracker()

loop_rate = rospy.Rate(30)

rospy.loginfo('Starting...')

while not rospy.is_shutdown():
    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('Hue Min', 'Blob HSV Calibrator')
    sMin = cv2.getTrackbarPos('Saturation Min', 'Blob HSV Calibrator')
    vMin = cv2.getTrackbarPos('Value Min', 'Blob HSV Calibrator')
    hMax = cv2.getTrackbarPos('Hue Max', 'Blob HSV Calibrator')
    sMax = cv2.getTrackbarPos('Saturation Max', 'Blob HSV Calibrator')
    vMax = cv2.getTrackbarPos('Value Max', 'Blob HSV Calibrator')

    if (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax):
        print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (
        hMin, sMin, vMin, hMax, sMax, vMax))
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

    # get an Image
    cv_image = camera_service.get_image()

    cv_image = rescaleFrame(cv_image, scale=0.8)

    cv2.imshow('Blurred Image', cv2.GaussianBlur(cv_image, (5, 5), 2.7))

    keypoints, mask = blob_tracker.blob_detect(cv_image, hsv_max=upper, hsv_min=lower, blur=5)

    cv2.imshow('Blob Mask', mask)

    sorted_keypoints = sorted(keypoints, reverse=True, key=lambda e: e.size)

    if len(sorted_keypoints) > 0:

        keypoint = sorted_keypoints[0]

        x, y = blob_tracker.get_blob_relative_position(cv_image, keypoint)
        blob_size = keypoint.size
        blob_tracker.publish_blob(x, y, blob_size)

        image_with_keypoints = blob_tracker.draw_keypoints(cv_image, [keypoint])

        cv2.imshow("Blob with keypoints", image_with_keypoints)

    # for keypoint in keypoints:
    #    x, y = blob_tracker.get_blob_relative_position(cv_image, keypoint)
    #    blob_size = keypoint.size
    #    blob_tracker.publish_blob(x, y, blob_size)
    #
    # image_with_keypoints = blob_tracker.draw_keypoints(cv_image, keypoints)
    # cv2.imshow("Blob with keypoints", image_with_keypoints)

    cv2.imshow('Blob HSV Calibrator', cv_image)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

    loop_rate.sleep()
