#ifndef AIP_BOSCH_GRIPPER
#define AIP_BOSCH_GRIPPER

#include "rclcpp/rclcpp.hpp"
#include <kuka_eki_io_interface/kuka_eki_io_interface.h>
#include <iras_interfaces/srv/move_gripper.hpp>

namespace aip_bosch_gripper
{
    class Gripper : public rclcpp::Node
    {
    public:
        Gripper();
        ~Gripper(){};

    private:
        std::string robot_ip_ = "robot_ip";
        std::shared_ptr<kuka_eki_io_interface::KukaEkiIOInterface> _kuka_eki_io_interface;
        rclcpp::Service<iras_interfaces::srv::MoveGripper>::SharedPtr _open_gripper_srv;
        rclcpp::Service<iras_interfaces::srv::MoveGripper>::SharedPtr _close_gripper_srv;
        void open_gripper(const std::shared_ptr<iras_interfaces::srv::MoveGripper::Request> request,
                          std::shared_ptr<iras_interfaces::srv::MoveGripper::Response> response);
        void close_gripper(const std::shared_ptr<iras_interfaces::srv::MoveGripper::Request> request,
                           std::shared_ptr<iras_interfaces::srv::MoveGripper::Response> response);
    };
}
#endif