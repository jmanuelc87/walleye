#include "walleye_control/walleye_hw_interface.hpp"

using namespace hardware_interface;

namespace walleye_hardware_interface
{

WalleyeHardwareInterface::WalleyeHardwareInterface(){};
WalleyeHardwareInterface::~WalleyeHardwareInterface(){};

bool WalleyeHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    ROS_INFO_NAMED("walleye_hardware_interface", "Registering...");

    JointStateHandle state_handle_lt("left_wheel", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_lt);

    JointStateHandle state_handle_rt("right_wheel", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_rt);

    JointStateHandle pan_handle_joint("pan_joint", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(pan_handle_joint);
    JointStateHandle tilt_handle_joint("tilt_joint", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(tilt_handle_joint);

    registerInterface(&jnt_state_interface);

    JointHandle vel_handle_lt(jnt_state_interface.getHandle("left_wheel"), &cmd[0]);
    jnt_velocity_interface.registerHandle(vel_handle_lt);

    JointHandle vel_handle_rt(jnt_state_interface.getHandle("right_wheel"), &cmd[1]);
    jnt_velocity_interface.registerHandle(vel_handle_rt);

    registerInterface(&jnt_velocity_interface);

    ROS_INFO_NAMED("walleye_hardware_interface", "Registering pan joint...");
    JointHandle pan_handle(jnt_state_interface.getHandle("pan_joint"), &cmd[2]);
    jnt_position_interface.registerHandle(pan_handle);

    ROS_INFO_NAMED("walleye_hardware_interface", "Registering tilt joint...");
    JointHandle tilt_handle(jnt_state_interface.getHandle("tilt_joint"), &cmd[3]);
    jnt_position_interface.registerHandle(tilt_handle);

    registerInterface(&jnt_position_interface);

    ROS_INFO_NAMED("walleye_hardware_interface", "Registering limits pan joint...");
    JointLimits pan_limits;
    getJointLimits("pan_joint", root_nh, pan_limits);

    PositionJointSaturationHandle pan_handle_limits(jnt_position_interface.getHandle("pan_joint"), pan_limits);
    jnt_limit_interface.registerHandle(pan_handle_limits);

    ROS_INFO_NAMED("walleye_hardware_interface", "Registering limits tilt joint...");
    JointLimits tilt_limits;
    bool tilt_ok = getJointLimits("tilt_joint", root_nh, tilt_limits);

    if (!tilt_ok) {
        ROS_INFO_NAMED("walleye_hardware_interface", "has pos limits %d, min: %f, max: %f", tilt_limits.has_position_limits, tilt_limits.min_position, tilt_limits.max_position);
        ROS_INFO_NAMED("walleye_hardware_interface", "no limits found.");
        ros::shutdown();
    }

    PositionJointSaturationHandle tilt_handle_limits(jnt_position_interface.getHandle("tilt_joint"), tilt_limits);
    jnt_limit_interface.registerHandle(tilt_handle_limits);

    registerInterface(&jnt_limit_interface);

    return true;
};

bool WalleyeHardwareInterface::open(ros::NodeHandle *root_nh)
{
    std::string ns = root_nh->getNamespace();
    std::string device = "/dev/ttyTHS1";
    int baud_rate = B115200;

    if (root_nh->hasParam(ns + "/hw/device")) {
        root_nh->getParam(ns + "/hw/device", device);
    }

    if (root_nh->hasParam(ns + "/robot_movement_controller/wheel_radius")) {
        root_nh->getParam(ns + "/robot_movement_controller/wheel_radius", radius);
    }

    this->servo = new drivers::ServoMotor(device.c_str(), baud_rate);

    try {
        this->servo->begin();
    }  catch (std::runtime_error &err) {
        ROS_ERROR_NAMED("walleye_hardware_interface", "Error while starting Servo Motor %s", err.what());
        ros::shutdown();
    }

    this->chip = new drivers::PCA9685(0x40, 1);

    try {
        chip->begin();
    }  catch (std::runtime_error &err) {
        ROS_ERROR_NAMED("walleye_hardware_interface", "Error while starting chip PCA9685 %s", err.what());
        ros::shutdown();
    }

    this->right_motor = new drivers::MotorDC(8, 9, chip);

    this->left_motor = new drivers::MotorDC(11, 10, chip);

    return true;
};

bool WalleyeHardwareInterface::close() {

    this->servo->end();
    delete this->servo;

    this->chip->end();
    delete this->right_motor;
    delete this->left_motor;
    delete this->chip;

    return true;
}

bool WalleyeHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
    // dummy feedback from the sensors & actuators
    vel[0] = cmd[0];
    vel[1] = cmd[1];
    pos[2] = cmd[2];
    pos[3] = cmd[3];

    return true;
};

/**
 * @brief adjustSpeed is a custom function to adjust the speed acording to a equation
 * @param w the angular speed
 * @param radius the radius of the wheel
 * @return the speed in PWM 12 bit resolution
 */
double adjustSpeedToPWM(double w, double radius) {
    return 2505.29396764859 * (w * radius) + 1099.1643862477;
}

bool WalleyeHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
    // enforce limits
    jnt_limit_interface.enforceLimits(period);
    
    // move pan servo
    if (pos[2] != cmd[2]) {
        
        ROS_INFO_NAMED("walleye_hardware_interface", "PAN %f %f",
                       jnt_position_interface.getHandle("pan_joint").getPosition(),
                       jnt_position_interface.getHandle("pan_joint").getCommand());
        try {
            usleep(100);
            this->servo->moveById(PAN_ID, jnt_position_interface.getHandle("pan_joint").getCommand());
        }  catch (std::domain_error &err) {
            ROS_ERROR_NAMED("walleye_hardware_interface", "Error in moveById %s", err.what());
            ros::shutdown();
        }
    }

    // move tilt servo
    if (pos[3] != cmd[3]) {
        ROS_INFO_NAMED("walleye_hardware_interface", "TILT %f %f",
                       jnt_position_interface.getHandle("tilt_joint").getPosition(),
                       jnt_position_interface.getHandle("tilt_joint").getCommand());

        try {
            usleep(100);
            this->servo->moveById(TILT_ID, jnt_position_interface.getHandle("tilt_joint").getCommand());
        }  catch (std::domain_error &err) {
            ROS_ERROR_NAMED("walleye_hardware_interface", "Error in moveById %s", err.what());
            ros::shutdown();
        }
    }

    // move left motor dc
    double speed = jnt_velocity_interface.getHandle("left_wheel").getCommand();
    double pwm_speed = adjustSpeedToPWM(speed, radius);

    try {
        usleep(100);
        if (speed > 0.0) {
            left_motor->run(pwm_speed, drivers::MotorDirection::FORWARD);
        } else if (speed < 0.0) {
            left_motor->run(pwm_speed, drivers::MotorDirection::BACKWARD);
        } else {
            left_motor->brake();
        }
    }  catch (std::runtime_error &err) {
        ROS_ERROR_NAMED("walleye_hardware_interface", "Error in left motor DC %s", err.what());
        ros::shutdown();
    }

    // move right motor dc
    speed = jnt_velocity_interface.getHandle("right_wheel").getCommand();
    pwm_speed = adjustSpeedToPWM(speed, radius);

    try {
        usleep(100);
        if (speed > 0.0) {
            right_motor->run(pwm_speed, drivers::MotorDirection::FORWARD);
        } else if (speed < 0.0) {
            right_motor->run(pwm_speed, drivers::MotorDirection::BACKWARD);
        } else {
            right_motor->brake();
        }
    }  catch (std::runtime_error &err) {
        ROS_ERROR_NAMED("walleye_hardware_interface", "Error in left motor DC %s", err.what());
        ros::shutdown();
    }

    return true;
};
}
