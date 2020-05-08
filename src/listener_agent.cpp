#include <rosrect-listener-agent/listener_agent.h>
using std::placeholders::_1;

cs_listener::cs_listener() 
: Node("rosrect_listener_agent_node")
{
  // Constructor

  // Pulling environment variables
  this->agent_type = std::getenv("AGENT_TYPE");
  this->robot_code = std::getenv("ROBOT_CODE");

  // Depending on ENV variable, communicate to user
  if (this->agent_type == "DB")
  {
    std::cout << "Subscribed to Listener Agent with DB Access..." << std::endl;
  }
  else
  {
    std::cout << "Subscribed to Listener Agent with direct rosout..." << std::endl;
  }

  // Create subscription
  this->rosout_subscription = this->create_subscription<rcl_interfaces::msg::Log>("rosout", std::bind(&cs_listener::log_callback, this, _1));
}

cs_listener::~cs_listener()
{
  // Destructor
  std::cout << "Unsubscribed from Listener Agent..." << std::endl;
}

void cs_listener::log_callback(const rcl_interfaces::msg::Log::SharedPtr rosmsg)
{
  // Callback that hands over message to State Manager
  // this->state_manager_instance.check_message(this->agent_type, this->robot_code, rosmsg);

  if (rosmsg->name != "rviz2")
  {
    std::cout << "Message received: " << rosmsg->msg << std::endl;
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