#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iostream>
#include <error_resolution_diagnoser_ros2/state_manager.h>
#include <chrono>

class cs_listener : public rclcpp::Node
{
    // This class provides the ROS node interface for the agent.

    std::string agent_type;                                                                  // DB or ROS agent
    std::string robot_code;                                                                  // UUID supplied during setup
    rclcpp::TimerBase::SharedPtr heartbeat_timer;                                            // ROS timer that is configured to with heartbeat_start as callback
    std::chrono::seconds heartrate;                                                          // Period duration for heartbeat_timer
    StateManager state_manager_instance;                                                     // State manager object that processes all incoming messages
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_subscription;           // Subscriber for rosout
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;                       // Subscriber for Odometry telemetry info
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub; // Subscriber for Pose telemetry info
    web::json::value telemetry;                                                              // JSON value that will be pushed as a part of event/status
    bool telemetry_ok;                                                                       // Used to check if telemetry subs have been setup

public:
    cs_listener();                                                                                 // Constructor to set up listener object
    ~cs_listener();                                                                                // Destructor
    void setup_telemetry();                                                                        // Sets up additional subscribers dynamically to populate telemetry
    void log_callback(const rcl_interfaces::msg::Log::SharedPtr);                                  // Listener callback that hands over the rosout message to state manager for processing
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr);                                  // Odometry callback for telemetry info
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);            // Pose callback for telemetry info
    void heartbeat_start();                                                                        // Utility method to setup the heartbeat_timer
    void heartbeat_log();                                                                          // Timer callback that logs heartbeat online
    void heartbeat_stop();                                                                         // Method that is called when node is shut down to log heartbeat offline
    web::json::value odom_to_json(const nav_msgs::msg::Odometry::SharedPtr);                       // Utility function to convert Odometry message to JSON
    web::json::value pose_to_json(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr); // Utility function to convert Pose message to JSON
};
