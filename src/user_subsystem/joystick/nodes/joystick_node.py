
import math
import rospy
from rospy.core import logdebug

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from joystick import Controller, ControlDefinition, EventListener, AxisListener


class TiltListener(EventListener):

    def __init__(self) -> None:
        self.message_pub = rospy.Publisher(
            "/walleye/tilt_position_controller/command", Float64, queue_size=10)
        self.tilt = [120.0, 150.0, 200.0]
        self.index = 0

    def onButtonDown(self) -> None:
        pass

    def onButtonUp(self) -> None:
        pass

    def onButtonPress(self) -> None:
        msg: Float64 = Float64(self.tilt[self.index])

        self.message_pub.publish(msg)

        self.index = (self.index + 1) % len(self.tilt)


class PositionMoveListener(AxisListener):

    def __init__(self) -> None:
        self.publisher = rospy.Publisher(
            '/walleye/robot_movement_controller/cmd_vel', Twist, queue_size=100)

    def onAxisMoveAction(self, x, y) -> None:
        norm = self.__calculate__norm(x, y)
        theta = self.__calculate_theta(x, y)

        twist = Twist()
        twist.angular.z = theta
        
        if y != 0:
            twist.linear.x = norm

        self.publisher.publish(twist)

    def __calculate__norm(self, x, y):
        if y < 0:
            return -1 * round(math.sqrt(math.pow(x, 2) + math.pow(y, 2)), 3)
        return round(math.sqrt(math.pow(x, 2) + math.pow(y, 2)), 3)

    def __calculate_theta(self, x, y):
        if x == 0 and y == 0:
            return 0
        else:
            return round(math.cos(math.atan2(y, x)), 3)


class PanListener(AxisListener):

    def __init__(self) -> None:
        self.message_pub = rospy.Publisher(
            "/walleye/pan_position_controller/command", Float64, queue_size=10)

    def __map(self, x, x1, x2, y1, y2):
        return (((x - x1) / (x2 - x1)) * (y2 - y1)) + y1

    def onAxisMoveAction(self, x, y) -> None:
        degress: float = round(self.__map(x, -1.0, 1.0, 0, 360), 3)

        msg: Float64 = Float64(degress)

        self.message_pub.publish(msg)


def main():
    rospy.init_node("joystick_node", log_level=rospy.DEBUG)

    ns = rospy.get_namespace()

    control = Controller("/walleye/joy")
    control.addListener(ControlDefinition.BUTTON_Y, TiltListener())
    control.addListener(ControlDefinition.LEFT_AXIS, PositionMoveListener())
    control.addListener(ControlDefinition.RIGHT_AXIS, PanListener())

    rospy.spin()


if __name__ == '__main__':
    main()
