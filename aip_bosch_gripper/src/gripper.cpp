#include <aip_bosch_gripper/gripper.h>
//#include <iras_interfaces/OpenGripper.srv>

using std::placeholders::_1;
using std::placeholders::_2;

namespace aip_bosch_gripper
{    //
    std::map<int, std::vector<int>> modeOfOperation;

    Gripper::Gripper() : Node("aip_bosch_gripper_node")
    {
                /* Initalisierung der Modes
        Mode 1: Alle Greifer ausfahren
        Mode 2: Greifer 2 und 4 (große)
        Mode 3: Greifer 1 und 3 (kleine)
        Mode 4: Greifer 3 und 4 (links)
        Mode 5: Greifer 1 und 2 (rechts)
        Mode 6: Greifer 1 ausfahren
        Mode 7: Greifer 2 ausfahren
        Mode 8: Greifer 3 ausfahren
        Mode 9: Greifer 4 ausfahren

        Mode 10: Greifer 1 einfahren
        Mode 11: Greifer 2 einfahren
        Mode 12: Greifer 3 einfahren
        Mode 13: Greifer 4 einfahren
        */


        // Put the values in the map 
        // Insert only the single cylinders 
        // build the other modes based on the single cylinders

        modeOfOperation[6].push_back(40);
        modeOfOperation[6].push_back(33);

        modeOfOperation[7].push_back(42);
        modeOfOperation[7].push_back(34);

        /*
        modeOfOperation[8].push_back(44);
        modeOfOperation[8].push_back(35);

        modeOfOperation[9].push_back(46);
        modeOfOperation[9].push_back(36);

        modeOfOperation[10].push_back(39);
        modeOfOperation[10].push_back(33);

        modeOfOperation[11].push_back(41);
        modeOfOperation[11].push_back(34);

        modeOfOperation[12].push_back(43);
        modeOfOperation[12].push_back(35);

        modeOfOperation[13].push_back(45);
        modeOfOperation[13].push_back(36);
        */

        this->declare_parameter("robot_ip", "10.166.32.145");
        this->declare_parameter("eki_io_port", 54601);
        this->declare_parameter("n_io", 2);

        std::string robot_ip;
        this->get_parameter("robot_ip", robot_ip);
        //this->get_parameter("robot_ip", "10.166.32.145");
        int eki_io_port;
        std::string eki_io_port_;
        this->get_parameter("eki_io_port", eki_io_port);
        eki_io_port_ = std::to_string(eki_io_port);
        int n_io;
        this->get_parameter("n_io", n_io);

        _kuka_eki_io_interface.reset(new kuka_eki_io_interface::KukaEkiIOInterface(robot_ip.c_str(), eki_io_port_.c_str(), n_io));

        _open_gripper_srv = this->create_service<iras_interfaces::srv::MoveGripper>("open_gripper", std::bind(&Gripper::open_gripper, this, _1, _2));
        _close_gripper_srv = this->create_service<iras_interfaces::srv::MoveGripper>("close_gripper", std::bind(&Gripper::close_gripper, this, _1, _2));
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "TEST");
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Ready to receive commands. TETTSTSTETTSTSTST");
        RCLCPP_INFO(rclcpp::get_logger("Eigener Output: IP: "), robot_ip.c_str());


        /*
                // Zugriff auf die Werte
                std::vector<int> mode12 = modeOfOperation["mode12"];

                for(int value : mode12) {
                    std::cout << value << std::endl;
                }
        */


    }

    void Gripper::open_gripper(const std::shared_ptr<iras_interfaces::srv::MoveGripper::Request> request,
                               std::shared_ptr<iras_interfaces::srv::MoveGripper::Response> response)
    {
        // Ausgabe der Werte
        //std::string requestStr = std::to_string(request->data);
        //std::string responseStr = std::to_string(response->success);

        //  RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Response: %s", responseStr.c_str());
        //  RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Request: %s", requestStr.c_str());
        
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "OPEN GRIPPER erreicht");

        rclcpp::Rate loop_rate(50);
        // const std::vector<int> set_io_pins_cmd{8, 7};

        
        std::vector<int> set_io_pins_cmd;
        set_io_pins_cmd.assign(modeOfOperation[7].begin(), modeOfOperation[7].end());

        std::ostringstream oss;
        for(int number : set_io_pins_cmd) {
            oss << number << ' ';
        }
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "set_io_pins_cmd: %s", oss.str().c_str());

        //RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"),modeOfOperation["mode6"].begin());
        // RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"),modeOfOperation["mode6"].end());

        //const std::vector<int> set_io_pins_cmd{36, 46};
        const std::vector<int> set_io_modes_cmd{
            2,
            2,
        };
        //const std::vector<bool> set_target_ios_cmd{false, true};
        const std::vector<bool> set_target_ios_cmd{true, false};

        bool f_pin_set = false;
        bool s_pin_set = false;
        bool f_command_received = false;
        bool s_command_received = false;
        bool f_mode_set = false;
        bool s_mode_set = false;
        bool command_received = false;
        std::vector<bool> io_states;
        std::vector<int> io_pins;
        std::vector<int> io_types;
        int buff_len;
        // TODO: add timeout und mache Abbruchbedingung rein
        while (!command_received && rclcpp::ok())
        {
            _kuka_eki_io_interface->eki_write_command(set_io_pins_cmd, set_io_modes_cmd, set_target_ios_cmd);
            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
            f_pin_set = io_pins[0] == set_io_pins_cmd[0];
            s_pin_set = io_pins[1] == set_io_pins_cmd[1];
            f_command_received = io_states[0] == set_target_ios_cmd[0];
            s_command_received = io_states[1] == set_target_ios_cmd[1];
            f_mode_set = io_types[0] == set_io_modes_cmd[0];
            s_mode_set = io_types[1] == set_io_modes_cmd[1];
            command_received = f_pin_set && s_pin_set && f_command_received && s_command_received && f_mode_set && s_mode_set;
            //const std::vector<bool> set_target_ios_cmd{false, true};
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Ende erste Schleife, Open Gripper erreicht");


        }
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Open Gripper: Erste Schleife beendet!");

        bool opened = false;
        // const std::vector<int> io_pins_cmd{7, 8};
        const std::vector<int> io_pins_cmd{42, 34};
        const std::vector<int> io_modes_cmd{1, 1};
        const std::vector<bool> target_ios_cmd{true, false};
        // TODO: add timeout
        while (!opened && rclcpp::ok())
        {
            _kuka_eki_io_interface->eki_write_command(io_pins_cmd, io_modes_cmd, target_ios_cmd);
            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
            f_pin_set = io_pins[0] == io_pins_cmd[0];
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Wert für Pin %d : %d", io_pins_cmd[0], io_pins[0]);
            s_pin_set = io_pins[1] == io_pins_cmd[1];
            f_command_received = io_states[0] == true;
            s_command_received = io_states[1] == false;
            f_mode_set = io_types[0] == io_modes_cmd[0];
            s_mode_set = io_types[1] == io_modes_cmd[1];
            opened = f_pin_set && s_pin_set && f_command_received && s_command_received && f_mode_set && s_mode_set;
        }
        response->success = true;
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Open Gripper: Funktionsaufruf beendet!");
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "---------------*******-----------------");

    }

    void Gripper::close_gripper(const std::shared_ptr<iras_interfaces::srv::MoveGripper::Request> request,
                                std::shared_ptr<iras_interfaces::srv::MoveGripper::Response> response)
    //void Gripper::close_gripper(std::shared_ptr<OpenGripperSrv::Request> request,
    //                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // Ausgabe der Werte
        //std::string requestStr = std::to_string(request -> message );
        //std::string responseStr = std::to_string(response);

        //RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Request: %s", requestStr);
        //RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Response: %s", responseStr.c_str());


        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "CLOSE GRIPPER erreicht");
        rclcpp::Rate loop_rate(50);
        // const std::vector<int> set_io_pins_cmd{36, 45};
        std::vector<int> set_io_pins_cmd;
        set_io_pins_cmd.assign(modeOfOperation[11].begin(), modeOfOperation[11].end());

        std::ostringstream oss;
        for(int number : set_io_pins_cmd) {
            oss << number << ' ';
        }
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "set_io_pins_cmd: %s", oss.str().c_str());



        const std::vector<int> set_io_modes_cmd{2, 2};
        const std::vector<bool> set_target_ios_cmd{true, false};

        bool f_pin_set = false;
        bool s_pin_set = false;
        bool f_command_received = false;
        bool s_command_received = false;
        bool f_mode_set = false;
        bool s_mode_set = false;
        bool command_received = false;
        std::vector<bool> io_states;
        std::vector<int> io_pins;
        std::vector<int> io_types;
        int buff_len;
        // TODO: add timeout
        while (!command_received && rclcpp::ok())
        {
            _kuka_eki_io_interface->eki_write_command(set_io_pins_cmd, set_io_modes_cmd, set_target_ios_cmd);
            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
            f_pin_set = io_pins[0] == set_io_pins_cmd[0];
            s_pin_set = io_pins[1] == set_io_pins_cmd[1];
            f_command_received = io_states[0] == set_target_ios_cmd[0];
            s_command_received = io_states[1] == set_target_ios_cmd[1];
            f_mode_set = io_types[0] == set_io_modes_cmd[0];
            s_mode_set = io_types[1] == set_io_modes_cmd[1];
            command_received = f_pin_set && s_pin_set && f_command_received && s_command_received && f_mode_set && s_mode_set;
        }
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Ende erreicht: Greifer sollte geschlossen sein.");
        bool closed = false;
        const std::vector<int> io_pins_cmd{36, 46};
        const std::vector<int> io_modes_cmd{1, 1};
        const std::vector<bool> target_ios_cmd{false, false};
        // TODO: add timeout
        while (!closed && rclcpp::ok())
        {
             RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Beginn 2te Schleife.");
            _kuka_eki_io_interface->eki_write_command(io_pins_cmd, io_modes_cmd, target_ios_cmd);
            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
            f_pin_set = io_pins[0] == io_pins_cmd[0];
            s_pin_set = io_pins[1] == io_pins_cmd[1];
            f_command_received = io_states[0] == false;
            s_command_received = io_states[1] == true;
            f_mode_set = io_types[0] == io_modes_cmd[0];
            s_mode_set = io_types[1] == io_modes_cmd[1];
            closed = f_pin_set && s_pin_set && f_command_received && s_command_received && f_mode_set && s_mode_set;
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Ende 2 Schleife erreicht: Greifer sollte geschlossen sein.");
        }
        response->success = true;
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Close Gripper: Funktionsaufruf beendet!");
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "---------------*******-----------------");

    }
}
