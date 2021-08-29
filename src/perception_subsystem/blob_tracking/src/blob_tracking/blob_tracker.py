import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point


class BlobTracker:

    def __init__(self):
        self.ns = rospy.get_namespace()
        self.point_blob_topic = self.ns + "blob/point_blob"
        # This publisher  uses Point message to publish
        # x,y: x,y relative poses of the center of the blob detected relative to the center of teh image
        # z: size of the blob detected
        self.pub_blob = rospy.Publisher(self.point_blob_topic, Point, queue_size=1)

    def blob_detect(self,
                    image,
                    hsv_min,
                    hsv_max,
                    blur=0,
                    blob_params=None,
                    search_window=None):
        """
        :param image: The frame
        :param hsv_min: minimum threshold of the hsv filter [h_min, s_min, v_min]
        :param hsv_max: maximum threshold of the hsv filter [h_max, s_max, v_max]
        :param blur: blur value (default 0)
        :param blob_params: blob parameters (default None)
        :param search_window: window where to search as [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
        :return:
        """

        image_u = cv2.UMat(image)

        # - Blur image to remove noise
        if blur > 0:
            blurred = cv2.GaussianBlur(image_u, (blur, blur))

        # - Search window
        if search_window is None:
            search_window = [0.0, 0.0, 1.0, 1.0]

        # - Convert image from BGR to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # - Apply HSV threshold
        mask = cv2.inRange(hsv, hsv_min, hsv_max)

        # - dilate makes the in range areas larger
        mask = cv2.dilate(mask, None, iterations=2)

        mask = cv2.UMat.get(mask)

        # - Cut the image using the search mask
        mask = self.apply_search_window(mask, search_window)

        # - build default blob detection parameters, if none have been provided
        if blob_params is None:
            params = self.get_default_blob_params()
        else:
            params = blob_params

        # - Apply blob detection
        detector = cv2.SimpleBlobDetector_create(params)

        # Reverse the mask: blobs are black on white
        # reversemask = 255 - mask

        keypoints = detector.detect(mask)

        return keypoints, mask

    def get_default_blob_params(self):
        # Set up the SimpleBlobdetector with default parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Detect light blobs
        params.blobColor = 255

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 100

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 30
        params.maxArea = 20000

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        return params

    def apply_search_window(self, image, window_adim=[0.0, 0.0, 1.0, 1.0]):
        """
        Apply search window
        :param image: standar opencv frame
        :param window_adim: box for searching
        :return:
        """
        rows = image.shape[0]
        cols = image.shape[1]
        x_min_px = int(cols * window_adim[0])
        y_min_px = int(rows * window_adim[1])
        x_max_px = int(cols * window_adim[2])
        y_max_px = int(rows * window_adim[3])

        # --- Initialize the mask as a black image
        mask = np.zeros(image.shape, np.uint8)

        # --- Copy the pixels from the original image corresponding to the window
        mask[y_min_px:y_max_px, x_min_px:x_max_px] = image[y_min_px:y_max_px, x_min_px:x_max_px]

        # --- return the mask
        return mask

    def draw_frame(self, image, dimension=0.3, line=2):
        """
        Draw X Y frame
        :param image:
        :param dimension:
        :param line:
        :return: image
        """

        rows = image.shape[0]
        cols = image.shape[1]
        size = min([rows, cols])
        center_x = int(cols / 2)
        center_y = int(rows / 2)

        line_length = int(size, dimension)

        # -- X
        image = cv2.line(image, (center_x, center_y), (center_x + line_length, center_y), (0, 0, 255), line)
        # -- Y
        image = cv2.line(image, (center_x, center_y), (center_x, center_y + line_length), (0, 255, 0), line)

        return image

    def get_blob_relative_position(self, image, keyPoint):
        """
        Obtain the camera relative frame coordinate of one single keypoint
        :param image:
        :param keyPoint:
        :return: x, y
        """
        rows = float(image.shape[0])
        cols = float(image.shape[1])

        center_x = 0.5 * cols
        center_y = 0.5 * rows

        x = (keyPoint.pt[0] - center_x) / center_x
        y = (keyPoint.pt[1] - center_y) / center_y

        return x, y

    def publish_blob(self, x, y, size):
        blob_point = Point()
        blob_point.x = x
        blob_point.y = y
        blob_point.z = size
        self.pub_blob.publish(blob_point)

    def draw_keypoints(self,
                       image,  # -- Input image
                       keypoints,  # -- CV keypoints
                       line_color=(0, 0, 255),  # -- line's color (b,g,r)
                       ):
        """
        Draw detected blobs: returns the image
        return(im_with_keypoints)
        """

        # -- Draw detected blobs as red circles.
        # -- cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), line_color,
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        for count, point in enumerate(keypoints):
            im_with_keypoints_text = cv2.putText(im_with_keypoints, "({}, {})".format(point.pt[0], point.pt[1]),
                                                 (10, count * 20),
                                                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return im_with_keypoints_text
