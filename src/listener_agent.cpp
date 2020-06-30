#include <rosrect-listener-agent/listener_agent.h>
using std::placeholders::_1;

cs_listener::cs_listener() 
: Node("rosrect_listener_agent_node")
{
  // Constructor

  // AGENT_TYPE
  if(std::getenv("AGENT_TYPE"))
  {
    // Success case
    this->agent_type = std::getenv("AGENT_TYPE");
    // See if configuration is correct otherwise default to ROS
    if((this->agent_type == "DB") || (this->agent_type == "ERT") || (this->agent_type == "ECS"))
    {
      if(std::getenv("ECS_API"))
      {
        // Success case
        if(std::getenv("ECS_ROBOT_MODEL"))
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
  if(std::getenv("ROBOT_CODE"))
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
  else if(this->agent_type == "ECS"){
    std::cout << "Subscribed to Listener Agent with ECS Access..." << std::endl;
  }
  else
  {
    std::cout << "Subscribed to Listener Agent with direct rosout..." << std::endl;
  }

  // Create subscription
  this->rosout_subscription = this->create_subscription<rcl_interfaces::msg::Log>("rosout", 5000, std::bind(&cs_listener::log_callback, this, _1));
}

cs_listener::~cs_listener()
{
  // Destructor
  std::cout << "Unsubscribed from Listener Agent..." << std::endl;
}

void cs_listener::log_callback(const rcl_interfaces::msg::Log::SharedPtr rosmsg)
{
  if (rosmsg->name != "rviz2")
  {
    std::cout << "Message received: " << rosmsg->msg << std::endl;
    // Callback that hands over message to State Manager
    this->state_manager_instance.check_message(this->agent_type, this->robot_code, rosmsg);
  }
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