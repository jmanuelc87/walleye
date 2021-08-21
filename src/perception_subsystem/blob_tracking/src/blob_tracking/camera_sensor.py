import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraSensor:

  def __init__(self):
    self.bridge_object = CvBridge()
    self.camera_topic = "/walleye/camera/raw"
    self.__check_cv_image_ready()
    self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.camera_callback)

  def __check_cv_image_ready(self):
    self.cv_image = None
    while self.cv_image is None and not rospy.is_shutdown():
      try:
        raw_cv_image = rospy.wait_for_message(self.camera_topic, Image, timeout=1.0)
        self.cv_image = self.bridge_object.imgmsg_to_cv2(raw_cv_image, desired_encoding="bgr8")
        rospy.logdebug("Current %s READY", self.camera_topic)
      except:
        rospy.logerr("Current image object not ready yet retrying for getting %s", self.camera_topic)

    return self.cv_image

  def camera_callback(self, data):
    try:
      self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr("Error while reading from topic and converting image %s", self.camera_topic)

  def get_image(self):
    return self.cv_image
