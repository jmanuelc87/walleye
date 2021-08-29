#! /usr/bin/env python3
import cv2
import numpy as np
import rospy
from blob_tracking import BlobTracker
from camera.service import ServiceImage


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
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

service_image = ServiceImage()
blob_tracker = BlobTracker()

rospy.init_node('blob_hsv_calibrator_node', log_level=rospy.DEBUG)

loop_rate = rospy.Rate(35)

while not rospy.is_shutdown():
    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin', 'Blob HSV Calibrator')
    sMin = cv2.getTrackbarPos('SMin', 'Blob HSV Calibrator')
    vMin = cv2.getTrackbarPos('VMin', 'Blob HSV Calibrator')
    hMax = cv2.getTrackbarPos('HMax', 'Blob HSV Calibrator')
    sMax = cv2.getTrackbarPos('SMax', 'Blob HSV Calibrator')
    vMax = cv2.getTrackbarPos('VMax', 'Blob HSV Calibrator')

    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # get an Image
    cv_image = service_image.get_frame(rescale=0.5)

    keypoints, mask = blob_tracker.blob_detect(cv_image, hsv_max=upper, hsv_min=lower, blur=5)

    cv2.imshow('Blob Mask', mask)

    for keypoint in keypoints:
        x, y = blob_tracker.get_blob_relative_position(cv_image, keypoint)
        blob_size = keypoint.size
        blob_tracker.publish_blob(x, y, blob_size)

    image_with_keypoints = blob_tracker.draw_keypoints(cv_image, keypoints)

    cv2.imshow('Blob HSV Calibrator', image_with_keypoints)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

    loop_rate.sleep()
