import rospy

from camera.srv import ServiceImage


class CameraService:

    def __init__(self):
        self.service_name = 'get_camera_picture'
        rospy.wait_for_service(self.service_name)
        self.get_camera_picture = rospy.SericeProxy(self.service_name, ServiceImage)

    def get_frame(self, rescale=1.0):
        frame = None
        try:
            srv = ServiceImage()
            srv.rescale = rescale
            frame = self.get_camera_picture(srv)
        except rospy.ServiceException as err:
            rospy.logerr("Error requesting service %s with %s", self.service_name, err)

        return frame.image
