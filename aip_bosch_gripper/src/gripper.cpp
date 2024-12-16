#include <aip_bosch_gripper/gripper.h>

#include <iostream>
#include <thread>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;

namespace aip_bosch_gripper
{       
    Gripper::Gripper() : Node("aip_bosch_gripper_node")
    {
        // Mapping of pins to the corresponding cylinders: 
        // structure: extend_pin, retract_pin, extend_check_pin, retract_check_pin, suction_pin
        // For reference, please review the mapping_hw_to_io_number.md in the aip_wiki GitHub repository
        cylinder_pins = {
            {1, {40, 39, 132, 134, 33}},
            {2, {42, 41, 136, 138, 34}},
            {3, {44, 43, 133, 135, 35}},
            {4, {46, 45, 137, 139, 36}}
        };

        rclcpp::Rate loop_rate(50);


        // Initalization of robot connection
        this->declare_parameter("robot_ip", "10.166.32.145");
        this->declare_parameter("eki_io_port", 54601);
        this->declare_parameter("n_io", 2);
        std::string robot_ip;
        this->get_parameter("robot_ip", robot_ip);

        int eki_io_port;
        std::string eki_io_port_;
        this->get_parameter("eki_io_port", eki_io_port);
        eki_io_port_ = std::to_string(eki_io_port);
        int n_io;
        this->get_parameter("n_io", n_io);

        _kuka_eki_io_interface.reset(new kuka_eki_io_interface::KukaEkiIOInterface(robot_ip.c_str(), eki_io_port_.c_str(), n_io));

        _open_gripper_srv = this->create_service<aip_interfaces::srv::MoveGripper>("open_gripper", std::bind(&Gripper::open_gripper, this, _1, _2));
        _close_gripper_srv = this->create_service<aip_interfaces::srv::MoveGripper>("close_gripper", std::bind(&Gripper::close_gripper, this, _1, _2));

        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Ready to receive commands. ");
        RCLCPP_INFO(rclcpp::get_logger("Eigener Output: IP: "), robot_ip.c_str());
    }


    void Gripper::close_gripper(const std::shared_ptr<aip_interfaces::srv::MoveGripper::Request> request,
                                std::shared_ptr<aip_interfaces::srv::MoveGripper::Response> response)
    {
               
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "CLOSE GRIPPER erreicht");


        std::vector<int> eject_pins;
        std::vector<int> retract_pins;
        std::vector<int> eject_check_pins;
        std::vector<int> retract_check_pins;
        std::vector<int> suction_pins;
        for (auto const& cylinder_id : request->cylinder_ids.cylinder_ids)
        {
            eject_pins.push_back(cylinder_pins[cylinder_id][0]);
            retract_pins.push_back(cylinder_pins[cylinder_id][1]);
            eject_check_pins.push_back(cylinder_pins[cylinder_id][2]);
            retract_check_pins.push_back(cylinder_pins[cylinder_id][3]);
            suction_pins.push_back(cylinder_pins[cylinder_id][4]);
        }
        
        //pins {c1_aus, c2_aus, c3_aus}
        if (eject_pins.size() % 2 != 0)
        {
            eject_pins.push_back(eject_pins[0]);
            retract_pins.push_back(retract_pins[0]);
            eject_check_pins.push_back(eject_check_pins[0]);
            retract_check_pins.push_back(retract_check_pins[0]);
            suction_pins.push_back(suction_pins[0]);
        }

        //pins {c1_aus, c2_aus, c3_aus, c1_aus}
        //for (std::vector<int>::size_type i = 0; i < eject_pins.size(); i+=2)
        for (int i = 0; i < eject_pins.size(); i+=2)
        {

            //RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Eject pin size: %d", eject_pins.size());
            
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Execute command on eject pin: %d", eject_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Execute command on eject pin: %d", eject_pins[i+1]);

            // eject cylinder
            execute_commands({eject_pins[i], eject_pins[i+1]}, {true, true});
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        

        for (int i = 0; i < eject_pins.size(); i+=2)
        {

            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Reset eject pin: %d", eject_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Reset eject pin: %d", eject_pins[i+1]);


            // check if cylinder is ejected
            check_commands({eject_check_pins[i], eject_check_pins[i+1]}, {true, true});  
            // reset the ejection pin command
            execute_commands({eject_pins[i], eject_pins[i+1]}, {false, false});  
        }

        for (int i = 0; i < suction_pins.size(); i+=2)
        {
            
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Start vacuum suction pin: %d", suction_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Start vacuum suction pin: %d", suction_pins[i+1]);

            // suction on 
            execute_commands({suction_pins[i], suction_pins[i+1]}, {true, true});   

        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        for (int i = 0; i < retract_pins.size(); i+=2)
        {
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Retract pin: %d", retract_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Retract pin: %d", retract_pins[i+1]);
            // retract cylinder
            execute_commands({retract_pins[i], retract_pins[i+1]}, {true, true});

        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        for (int i = 0; i < retract_pins.size(); i+=2)
        {
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Reset retraction pin: %d", eject_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Reset retraction pin: %d", eject_pins[i+1]);

            // check if cylinder is retracted
            check_commands({retract_check_pins[i], retract_check_pins[i+1]}, {true, true});
            // reset the retraction pin command
            execute_commands({retract_pins[i], retract_pins[i+1]}, {false, false});
        }

        response->success = true;
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "CLOSE GRIPPER: Funktionsaufruf beendet!");
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "---------------*******-----------------");

    }


    void Gripper::open_gripper(const std::shared_ptr<aip_interfaces::srv::MoveGripper::Request> request,
                               std::shared_ptr<aip_interfaces::srv::MoveGripper::Response> response)
    {
        
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "OPEN GRIPPER erreicht");


        std::vector<int> eject_pins;
        std::vector<int> retract_pins;
        std::vector<int> eject_check_pins;
        std::vector<int> retract_check_pins;
        std::vector<int> suction_pins;
        for (auto const& cylinder_id : request->cylinder_ids.cylinder_ids)
        {
            eject_pins.push_back(cylinder_pins[cylinder_id][0]);
            retract_pins.push_back(cylinder_pins[cylinder_id][1]);
            eject_check_pins.push_back(cylinder_pins[cylinder_id][2]);
            retract_check_pins.push_back(cylinder_pins[cylinder_id][3]);
            suction_pins.push_back(cylinder_pins[cylinder_id][4]);
        }
        
        //pins {c1_aus, c2_aus, c3_aus}
        if (eject_pins.size() % 2 != 0)
        {
            eject_pins.push_back(eject_pins[0]);
            retract_pins.push_back(retract_pins[0]);
            eject_check_pins.push_back(eject_check_pins[0]);
            retract_check_pins.push_back(retract_check_pins[0]);
            suction_pins.push_back(suction_pins[0]);
        }

        //pins {c1_aus, c2_aus, c3_aus, c1_aus}
        //for (std::vector<int>::size_type i = 0; i < eject_pins.size(); i+=2)
        for (int i = 0; i < eject_pins.size(); i+=2)
        {

            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Execute command on eject pin: %d", eject_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Execute command on eject pin: %d", eject_pins[i+1]);

            // eject cylinder
            execute_commands({eject_pins[i], eject_pins[i+1]}, {true, true});
        }

            std::this_thread::sleep_for(std::chrono::seconds(1));

        for (int i = 0; i < eject_pins.size(); i+=2)
        {

            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Reset eject pin: %d", eject_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Reset eject pin: %d", eject_pins[i+1]);


            // check if cylinder is ejected
            check_commands({eject_check_pins[i], eject_check_pins[i+1]}, {true, true});  // ##################
            // reset the ejection pin command
            execute_commands({eject_pins[i], eject_pins[i+1]}, {false, false});  
        }

        for (int i = 0; i < suction_pins.size(); i+=2)
        {
            
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Start vacuum suction pin: %d", suction_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Start vacuum suction pin: %d", suction_pins[i+1]);

            // suction on 
            execute_commands({suction_pins[i], suction_pins[i+1]}, {false, false});   

        }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        
        for (int i = 0; i < retract_pins.size(); i+=2)
        
        {
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Retract pin: %d", retract_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Retract pin: %d", retract_pins[i+1]);
            // retract cylinder
            execute_commands({retract_pins[i], retract_pins[i+1]}, {true, true});

        }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        
        for (int i = 0; i < retract_pins.size(); i+=2)
        {
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Reset retraction pin: %d", eject_pins[i]);
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "Reset retraction pin: %d", eject_pins[i+1]);

            // check if cylinder is retracted
            check_commands({retract_check_pins[i], retract_check_pins[i+1]}, {true, true}); // ###################
            // reset the retraction pin command
            execute_commands({retract_pins[i], retract_pins[i+1]}, {false, false});
        }

        response->success = true;
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "OPEN GRIPPER: Funktionsaufruf beendet!");
        RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "---------------*******-----------------");

    }

    void Gripper::execute_commands(const std::vector<int>& pins, const std::vector<bool>& values)
    {
    
        rclcpp::Rate loop_rate(50);

        // static variables for eki_write_command (SetOutput -> see EkiIOInterface.xml)
        const std::vector<int> set_io_types{2, 2};

        // variables for eki_read_state
        int buff_len;
        std::vector<bool> io_states;
        std::vector<int> io_pins;
        std::vector<int> io_types;

        // declaration of comparison valiables
        bool f_command_received = false;
        bool s_command_received = false;
        bool f_mode_set = false;
        bool s_mode_set = false;

        bool command_received = false;

        while (!command_received && rclcpp::ok())
        {
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "In execute_commands - While loop");

            _kuka_eki_io_interface->eki_write_command(pins, set_io_types, values);

            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
                   
            f_command_received = io_states[0] == values[0];
            s_command_received = io_states[1] == values[1];
            f_mode_set = io_types[0] == set_io_types[0];
            s_mode_set = io_types[1] == set_io_types[1];
           
            command_received = f_command_received && s_command_received && f_mode_set && s_mode_set;
            
        }
    }

    void Gripper::check_commands(const std::vector<int>& pins, const std::vector<bool>& values)
    {
      
        rclcpp::Rate loop_rate(50);

        // static variables for eki_write_command (GetInput -> see EkiIOInterface.xml)
        const std::vector<int> set_io_types{1, 1};
        
        // variables for eki_read_state
        std::vector<bool> io_states;
        std::vector<int> io_pins;
        std::vector<int> io_types;
        int buff_len;

        // declaration of comparison valiables
        bool f_command_received = false;
        bool s_command_received = false;
        bool f_mode_set = false;
        bool s_mode_set = false;

        bool opened = false;

        while (!opened && rclcpp::ok())
        {
            RCLCPP_INFO(rclcpp::get_logger("aip_bosch_gripper_node"), "In check_commands - While loop");
            _kuka_eki_io_interface->eki_write_command(pins, set_io_types, values);
            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
       
            f_command_received = io_states[0] == false;   
            s_command_received = io_states[1] == false;
           
            f_mode_set = io_types[0] == set_io_types[0]; 
            s_mode_set = io_types[1] == set_io_types[1];  
            
            opened = f_command_received && s_command_received && f_mode_set && s_mode_set;

        }
        
    }
}