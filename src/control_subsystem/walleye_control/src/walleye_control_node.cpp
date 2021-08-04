#include <cstring>

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "walleye_control/walleye_hw_interface.hpp"


using namespace controller_manager;
using namespace walleye_hardware_interface;


int main(int argc, char *argv[])
{
    ROS_INFO_ONCE_NAMED("walleye_control_node", "Starting control node...");

    ros::init(argc, argv, "walleye_control_node");

    ros::AsyncSpinner spinner(2);

    spinner.start();

    ros::NodeHandle root_nh;

    ROS_INFO_ONCE_NAMED("walleye_control_node", "Acquiring parameters...");

    std::string param = root_nh.getNamespace() + "/node/loop_rate";
    int loop_rate = 0;

    if (root_nh.hasParam(param)) {
        root_nh.getParam(param, loop_rate);
    }

    ROS_INFO_ONCE_NAMED("walleye_control_node", "%s: %d", param.c_str(), loop_rate);

    WalleyeHardwareInterface hw;
    ControllerManager controller(&hw, root_nh);

    hw.open(&root_nh);
    hw.init(root_nh, root_nh);

    ros::Rate loop(loop_rate);

    ros::Time timestamp;
    ros::Duration period;

    ROS_INFO_ONCE_NAMED("walleye_control_node", "Everything seems ok.");

    while (ros::ok())
    {
        timestamp = ros::Time::now();
        period = ros::Time::now() - timestamp;
        hw.read(timestamp, period);
        controller.update(timestamp, period);
        hw.write(timestamp, period);
        loop.sleep();
    }

    ROS_INFO_ONCE_NAMED("walleye_control_node", "Exiting...");

    hw.close();
    spinner.stop();
    
    return 0;
}
