#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <iostream>
#include <rosrect-listener-agent/state_manager.h>
#include <chrono>

class cs_listener : public rclcpp::Node
{

    std::string agent_type;                                                        // DB or ROS agent
    std::string robot_code;                                                        // UUID supplied during setup
    rclcpp::TimerBase::SharedPtr heartbeat_timer;                                  // ROS timer that is configured to with heartbeat_start as callback
    std::chrono::seconds heartrate;                                                // Period duration for heartbeat_timer
    StateManager state_manager_instance;                                           // State manager object that processes all incoming messages
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_subscription; // Subscriber for rosout

public:
    cs_listener();                                                // Constructor to set up listener object
    ~cs_listener();                                               // Destructor
    void log_callback(const rcl_interfaces::msg::Log::SharedPtr); // Listener callback that hands over the rosout message to state manager for processing
    void heartbeat_start();                                       // Utility method to setup the heartbeat_timer
    void heartbeat_log();                                         // Timer callback that logs heartbeat online
    void heartbeat_stop();                                        // Method that is called when node is shut down to log heartbeat offline
};