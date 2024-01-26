#include "rclcpp/rclcpp.hpp"
#include "aip_bosch_gripper/gripper.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aip_bosch_gripper::Gripper>());
    rclcpp::shutdown();
    return 0;
}