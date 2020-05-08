#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <iostream>
// #include <rosrect-listener-agent/state_manager.h>

class cs_listener : public rclcpp::Node
{

    std::string agent_type; // DB or ROS agent
    std::string robot_code; // UUID supplied during setup
    // StateManager state_manager_instance; // State manager object that processes all incoming messages
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_subscription;

public:
    cs_listener();                                                // Constructor to set up listener object
    ~cs_listener();                                               // Destructor
    void log_callback(const rcl_interfaces::msg::Log::SharedPtr); // Listener callback that hands over the rosout message to state manager for processing
};