#include <error_resolution_diagnoser_ros2/listener_agent.h>
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace web::json; // JSON features
using namespace web;       // Common features like URIs.

cs_listener::cs_listener() 
: Node("error_resolution_diagnoser_ros2")
{
  // Constructor

  // AGENT_TYPE
  if (std::getenv("AGENT_TYPE"))
  {
    // Success case
    this->agent_type = std::getenv("AGENT_TYPE");
    // See if configuration is correct otherwise default to ROS
    if ((this->agent_type == "DB") || (this->agent_type == "ERT") || (this->agent_type == "ECS"))
    {
      if (std::getenv("ECS_API"))
      {
        // Success case
        if (std::getenv("ECS_ROBOT_MODEL"))
        {
          // Success case
        }
        else
        {
          // Failure case - Default
          this->agent_type = "ROS";
        }
      }
      else
      {
        // Failure case - Default
        this->agent_type = "ROS";
      }
    }
  }
  else
  {
    // Failure case - Default
    this->agent_type = "ROS";
  }

  // ROBOT_CODE
  if (std::getenv("ROBOT_CODE"))
  {
    // Success case
    this->robot_code = std::getenv("ROBOT_CODE");
  }
  else
  {
    // Failure case - Default
    this->robot_code = "Undefined";
  }

  // Depending on ENV variable, communicate to user
  if ((this->agent_type == "DB") || (this->agent_type == "ERT"))
  {
    std::cout << "Subscribed to Listener Agent with ERT Access..." << std::endl;
  }
  else if (this->agent_type == "ECS")
  {
    std::cout << "Subscribed to Listener Agent with ECS Access..." << std::endl;
  }
  else
  {
    std::cout << "Subscribed to Listener Agent with direct rosout..." << std::endl;
  }

  // Heartbeat parameters
  this->heartrate = 15s;

  // Initiate heartbeat
  this->heartbeat_start();

  // Telemetry
  // Create JSON object
  this->telemetry = json::value::object();
  this->setup_telemetry();

  // Create rosout subscription
  this->rosout_subscription = this->create_subscription<rcl_interfaces::msg::Log>("rosout", 5000, std::bind(&cs_listener::log_callback, this, _1));
}

cs_listener::~cs_listener()
{
  // Stop heartbeat
  this->heartbeat_stop();
  // Destructor
  std::cout << "Unsubscribed from Listener Agent..." << std::endl;
}

json::value cs_listener::odom_to_json(const nav_msgs::msg::Odometry::SharedPtr rosmsg)
{
  // Create JSON objects
  json::value odom_json = json::value::object();
  json::value orientation = json::value::object();
  json::value position = json::value::object();

  // Create keys
  utility::string_t oKey(utility::conversions::to_string_t("orientation"));
  utility::string_t pKey(utility::conversions::to_string_t("position"));
  utility::string_t wKey(utility::conversions::to_string_t("w"));
  utility::string_t xKey(utility::conversions::to_string_t("x"));
  utility::string_t yKey(utility::conversions::to_string_t("y"));
  utility::string_t zKey(utility::conversions::to_string_t("z"));

  // Assign orientation key-value
  orientation[wKey] = json::value::number(rosmsg->pose.pose.orientation.w);
  orientation[xKey] = json::value::number(rosmsg->pose.pose.orientation.x);
  orientation[yKey] = json::value::number(rosmsg->pose.pose.orientation.y);
  orientation[zKey] = json::value::number(rosmsg->pose.pose.orientation.z);

  // Assign position key-value
  position[xKey] = json::value::number(rosmsg->pose.pose.position.x);
  position[yKey] = json::value::number(rosmsg->pose.pose.position.y);
  position[zKey] = json::value::number(rosmsg->pose.pose.position.z);

  // Assign odom key-value
  odom_json[oKey] = orientation;
  odom_json[pKey] = position;

  return (odom_json);
}

json::value cs_listener::pose_to_json(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr rosmsg)
{
  // Create JSON objects
  json::value pose_json = json::value::object();
  json::value orientation = json::value::object();
  json::value position = json::value::object();

  // Create keys
  utility::string_t oKey(utility::conversions::to_string_t("orientation"));
  utility::string_t pKey(utility::conversions::to_string_t("position"));
  utility::string_t wKey(utility::conversions::to_string_t("w"));
  utility::string_t xKey(utility::conversions::to_string_t("x"));
  utility::string_t yKey(utility::conversions::to_string_t("y"));
  utility::string_t zKey(utility::conversions::to_string_t("z"));

  // Assign orientation key-value
  orientation[wKey] = json::value::number(rosmsg->pose.pose.orientation.w);
  orientation[xKey] = json::value::number(rosmsg->pose.pose.orientation.x);
  orientation[yKey] = json::value::number(rosmsg->pose.pose.orientation.y);
  orientation[zKey] = json::value::number(rosmsg->pose.pose.orientation.z);

  // Assign position key-value
  position[xKey] = json::value::number(rosmsg->pose.pose.position.x);
  position[yKey] = json::value::number(rosmsg->pose.pose.position.y);
  position[zKey] = json::value::number(rosmsg->pose.pose.position.z);

  // Assign odom key-value
  pose_json[oKey] = orientation;
  pose_json[pKey] = position;

  return (pose_json);
}

void cs_listener::setup_telemetry()
{
  // Get all topics
  // auto all_topics = this->get_topic_names_and_types();
  // std::cout << "All topics:" << std::endl;
  // for (auto elem : all_topics)
  // {
  //   std::cout << elem.first << std::endl;
  //   for (auto type_info : elem.second)
  //   {
  //     std::cout << "\t" << type_info << std::endl;
  //   }
  // }
  // Find topics relevant to telemetry info and dynamically subscribe
  std::string odom_msg_type = "nav_msgs/Odometry";
  std::string pose_msg_type = "geometry_msgs/PoseWithCovarianceStamped";
  std::string odom_topic = "odom";
  std::string pose_topic = "amcl_pose";

  this->odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 5000, std::bind(&cs_listener::odom_callback, this, _1));
  this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 5000, std::bind(&cs_listener::pose_callback, this, _1));
}

void cs_listener::log_callback(const rcl_interfaces::msg::Log::SharedPtr rosmsg)
{

  if (rosmsg->name != "rviz2")
  {
    std::cout << "Message received: " << rosmsg->msg << std::endl;
    // Callback that hands over message to State Manager
    this->state_manager_instance.check_message(this->agent_type, this->robot_code, rosmsg, this->telemetry);
  }
}

void cs_listener::odom_callback(const nav_msgs::msg::Odometry::SharedPtr rosmsg)
{
  // Process odometry information for telemetry
  // std::cout << "Odom callback called" << std::endl;

  // Convert to JSON
  json::value odom_data = this->odom_to_json(rosmsg);

  // Create key
  utility::string_t odomKey(utility::conversions::to_string_t("odom_pose"));

  // Assign key-value
  this->telemetry[odomKey] = odom_data;
}

void cs_listener::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr rosmsg)
{
  // Process pose information for telemetry
  // std::cout << "Pose callback called" << std::endl;

  // Convert to JSON
  json::value pose_data = this->pose_to_json(rosmsg);

  // Create key
  utility::string_t poseKey(utility::conversions::to_string_t("nav_pose"));

  // Assign key-value
  this->telemetry[poseKey] = pose_data;
}

void cs_listener::heartbeat_start()
{
  // Records heartbeat online status when node is started. Future status is pushed by timer bound callback
  this->state_manager_instance.check_heartbeat(true, this->telemetry);

  // Create a Wall Timer for heartrate period
  this->heartbeat_timer = this->create_wall_timer(this->heartrate, std::bind(&cs_listener::heartbeat_log, this));
}

void cs_listener::heartbeat_log()
{
  // A timer bound method that periodically checks the ROS connection status and passes it to the state manager.
  bool status = rclcpp::ok();
  this->state_manager_instance.check_heartbeat(status, this->telemetry);
}

void cs_listener::heartbeat_stop()
{
  // Records heartbeat offline status when node is shutdown
  this->state_manager_instance.check_heartbeat(false, this->telemetry);
}

int main(int argc, char **argv)
{

  // Initialize node
  rclcpp::init(argc, argv);

  // Spin node
  rclcpp::spin(std::make_shared<cs_listener>());

  // Shut down
  rclcpp::shutdown();

  return 0;
}
