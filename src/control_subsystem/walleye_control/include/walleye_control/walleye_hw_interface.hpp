#ifndef WALLEYE_HARDWARE_INTERFACE__WALLEYE_HARDWARE_INTERFACE_HPP_
#define WALLEYE_HARDWARE_INTERFACE__WALLEYE_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/robot_hw.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"

#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "drivers/Motor.hpp"
#include "drivers/PCA9685.hpp"

#include "std_msgs/Float64MultiArray.h"

#include <limits>
#include <vector>
#include <string>
#include <cstdlib>
#include <termios.h>
#include <cmath>

#define PAN_ID 1
#define TILT_ID 2


using namespace hardware_interface;
using namespace joint_limits_interface;


namespace walleye_hardware_interface
{
class WalleyeHardwareInterface : public RobotHW
{
private:
    JointStateInterface jnt_state_interface;

    VelocityJointInterface jnt_velocity_interface;

    PositionJointInterface jnt_position_interface;

    PositionJointSaturationInterface jnt_limit_interface;

    double cmd[4];
    double pos[4];
    double vel[4];
    double eff[4];
    double radius;

    drivers::PCA9685 * chip;

    // controlling the movement of the robot
    drivers::MotorDC * left_motor;
    drivers::MotorDC * right_motor;

    // controlling the mount of the camera
    drivers::ServoMotor * servo;

public:
    WalleyeHardwareInterface();
    ~WalleyeHardwareInterface();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);

    bool open(ros::NodeHandle *root_nh);

    bool close();

    bool read(const ros::Time time, const ros::Duration period);

    bool write(const ros::Time time, const ros::Duration period);
};
}

#endif
