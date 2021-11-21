#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64


class BlobFollower:

    def __init__(self):
        rospy.loginfo("Initializing Blob Follower...")

        self.move_rate = rospy.Rate(10)
        self.acceptable_error = 0.185

        self.current_yaw = 180.0
        self.current_pitch = 150.0
        self.pan_obj = Float64(self.current_yaw)
        self.tilt_obj = Float64(self.current_pitch)
        self.pub_pan_command = rospy.Publisher("/walleye/pan_position_controller/command", Float64, queue_size=1)
        self.pub_tilt_command = rospy.Publisher("/walleye/tilt_position_controller/command", Float64, queue_size=1)

        self.ns = rospy.get_namespace()

        self.point_blob_topic = self.ns + "blob/point_blob"
        self.point_blob_service = self.ns + "blob/point_reset_service"
        self._check_blob_point_ready()
        rospy.Subscriber(self.point_blob_topic, Point, self.point_blob_callback)
        rospy.Service(self.point_blob_service, Point, self.point_reset_blob_service_callback)
        rospy.loginfo("Started: yaw: %s, pitch: %s", self.current_yaw, self.current_pitch)

    def _check_blob_point_ready(self):
        self.point_blob = None
        while self.point_blob is None and not rospy.is_shutdown():
            try:
                self.point_blob = rospy.wait_for_message(self.point_blob_topic, Point, timeout=1.0)
                rospy.logdebug("Current %s READY=>", self.point_blob_topic)
            except:
                rospy.logerr(
                    "Current %s not ready yet, retrying for getting %s", self.point_blob_topic, self.point_blob_topic)
        return self.point_blob

    def point_blob_callback(self, msg):
        rospy.logdebug("(%s,%s,%s)", msg.x, msg.y, msg.z)

        if msg.x > self.acceptable_error:
            self.pan_obj.data += 1.0
        elif msg.x < -1 * self.acceptable_error:
            self.pan_obj.data -= 1.0

        if msg.y > self.acceptable_error:
            self.tilt_obj.data += 1.0
        elif msg.y < -1 * self.acceptable_error:
            self.tilt_obj.data -= 1.0

        rospy.loginfo("yaw: %s, pitch: %s", self.current_yaw, self.current_pitch)

    def point_reset_blob_service_callback(self, msg):
        # ignore message for now
        rospy.loginfo("Reset pitch and yaw angles")
        self.current_pitch = 150.0
        self.current_yaw = 180.0

    def loop(self):
        while not rospy.is_shutdown():
            self.pub_pan_command.publish(self.pan_obj)
            self.pub_tilt_command.publish(self.tilt_obj)
            self.move_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("blob_follower_node", log_level=rospy.DEBUG)
    camera_join_follower = BlobFollower()
    camera_join_follower.loop()
