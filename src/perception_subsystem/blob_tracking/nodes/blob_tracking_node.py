import rospy

from blob_tracking import BlobTracker

rospy.init_node("blob_tracking_node", log_level=rospy.DEBUG)

blob_tracker = BlobTracker()

# HSV limits
hsv_min = (0, 234, 0)
hsv_max = (0, 255, 255)

# We define the detection area [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
window = [0.1, 0.1, 0.9, 0.9]

while not rospy.is_shutdown():
  # Get most recent image
  cv_image = blob_tracker.get_image()

  # Detect blobs
  keypoints, _ = blob_tracker.blob_detect(cv_image, hsv_min, hsv_max, blur=3, blob_params=None,
                                          search_window=window)

  for keypoint in keypoints:
    x, y = blob_tracker.get_blob_relative_position(cv_image, keypoint)
    blob_size = keypoint.size

    blob_tracker.publish_blob(x, y, blob_size)

rospy.logwarn("Shutting down")
