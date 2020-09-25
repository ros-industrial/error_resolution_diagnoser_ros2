#include <gtest/gtest.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <error_resolution_diagnoser_ros2/state_manager.h>

using namespace web::json; // JSON features
using namespace web;       // Common features like URIs.

// Create test object
StateManager state_manager_instance;

// Log file settings
std::string log_name;
std::string log_ext = ".json";
int log_id = 0;

// Sample message
std::string errorMessage = "Navigation failed";
std::string warningMessage = "Planning algorithm failed to generate a valid path";
std::string infoMessage = "Received a goal, begin following path";
std::string infoEndMessage = "Goal reached";

// Sample telemetry
json::value telemetry = json::value::parse("{ \"pose\" : 42 }");

// Sample log
std::vector<std::string> found;

// Utility functions
void setLogFolder()
{
  // Set log folder
  std::ifstream inFile;
  std::string home_var = std::getenv("HOME");
  std::string latest_log = home_var + "/.cognicept/agent/logs/latest_log_loc.txt";
  inFile.open(latest_log);
  if (!inFile)
  {
    std::cout << "Unable to open log file";
    exit(1); // terminate with error
  }
  else
  {
    inFile >> log_name;
    log_name.replace(0, 6, home_var);
    std::cout << "Reading logs from: " << log_name << std::endl;
  }
  inFile.close();
  log_name.append("/logData");
}

// Test cases
TEST(StateManagerTestSuite, existTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is sample text";

  // Check if message exists
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkErrorTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is a sample error text";

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Call check_warning
  state_manager_instance.check_error(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeFirstCheck = found.size();

  // Expecting first element of internal data structure to be robot code and next to be sample message
  ASSERT_EQ(found[0], sampleRobotCode);
  ASSERT_EQ(found[1], sampleMsgText);

  // Call check_warning again to see if suppression works. This should not add a new row.
  state_manager_instance.check_error(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeSecondCheck = found.size();

  // Check if size is the same
  ASSERT_EQ(sizeFirstCheck, sizeSecondCheck);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkWarningTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is a sample error text";

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Call check_warning
  state_manager_instance.check_warning(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeFirstCheck = found.size();

  // Expecting first element of internal data structure to be robot code and next to be sample message
  ASSERT_EQ(found[0], sampleRobotCode);
  ASSERT_EQ(found[1], sampleMsgText);

  // Call check_warning again to see if suppression works. This should not add a new row.
  state_manager_instance.check_warning(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeSecondCheck = found.size();

  // Check if size is the same
  ASSERT_EQ(sizeFirstCheck, sizeSecondCheck);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkInfoTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is a sample error text";

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Call check_warning
  state_manager_instance.check_info(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeFirstCheck = found.size();

  // Expecting first element of internal data structure to be robot code and next to be sample message
  ASSERT_EQ(found[0], sampleRobotCode);
  ASSERT_EQ(found[1], sampleMsgText);

  // Call check_warning again to see if suppression works. This should not add a new row.
  state_manager_instance.check_info(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeSecondCheck = found.size();

  // Check if size is the same
  ASSERT_EQ(sizeFirstCheck, sizeSecondCheck);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkMessageROSErrorTest)
{
  // Sample error message
  std::string sampleRobotCode = "SampleRobotCode";
  rcl_interfaces::msg::Log data;
  data.level = data.ERROR;
  data.name = "bt_navigator";
  data.msg = errorMessage;
  rcl_interfaces::msg::Log::SharedPtr rosmsg(new rcl_interfaces::msg::Log(data));

  // Call check_message_ros
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;
  std::cout << "Checking: " << filename << std::endl;

  // Check if file exists
  boost::filesystem::path infile1(filename);
  bool fileflag = boost::filesystem::exists(infile1);
  ASSERT_TRUE(fileflag);

  // Call check_message_ros again with the same message
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  filename = log_name + std::to_string(log_id) + log_ext;
  std::cout << "Checking: " << filename << std::endl;

  // Check if file exists
  boost::filesystem::path infile2(filename);
  fileflag = boost::filesystem::exists(infile2);
  ASSERT_TRUE(fileflag);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkMessageROSWarningTest)
{
  // Sample warning message
  std::string sampleRobotCode = "SampleRobotCode";
  rcl_interfaces::msg::Log data;
  data.level = data.WARN;
  data.name = "bt_navigator";
  data.msg = warningMessage;
  rcl_interfaces::msg::Log::SharedPtr rosmsg(new rcl_interfaces::msg::Log(data));

  // Call check_message_ros
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  boost::filesystem::path infile1(filename);
  bool fileflag = boost::filesystem::exists(infile1);
  ASSERT_TRUE(fileflag);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkMessageROSInfoTest)
{
  // Sample warning message
  std::string sampleRobotCode = "SampleRobotCode";
  rcl_interfaces::msg::Log data;
  data.level = data.INFO;
  data.name = "bt_navigator";
  data.msg = infoMessage;
  rcl_interfaces::msg::Log::SharedPtr rosmsg1(new rcl_interfaces::msg::Log(data));

  // Call check_message_ros
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg1, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file DOES NOT exist, since for info, the listener waits to push
  boost::filesystem::path infile1(filename);
  bool fileflag = boost::filesystem::exists(infile1);
  ASSERT_TRUE(fileflag);

  // Change message to an INFO end
  data.msg = infoEndMessage;
  rcl_interfaces::msg::Log::SharedPtr rosmsg2(new rcl_interfaces::msg::Log(data));

  // Call check_message_ros
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg2, telemetry);

  // Check if file exists, since for info end, the state manager pushes
  log_id++;
  filename = log_name + std::to_string(log_id) + log_ext;
  boost::filesystem::path infile2(filename);
  fileflag = boost::filesystem::exists(infile2);
  ASSERT_TRUE(fileflag);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, clearTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is a sample error text";

  // Call check_error
  state_manager_instance.check_error(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeFirstCheck = found.size();

  // Check if size is 1
  ASSERT_EQ(sizeFirstCheck, 3);

  // Clear state manager
  state_manager_instance.clear();

  // See if NO message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Check if result is empty
  ASSERT_EQ(found[0], "");
}

// TEST(StateManagerTestSuite, checkMessageDBErrorTest)
// {
//   // Sample error message
//   std::string sampleRobotCode = "SampleRobotCode";
//   rosgraph_msgs::Log data;
//   data.level = 8;
//   data.name = "/move_base";
//   data.msg = errorMessage;
//   rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

//   // Call check_message_db
//   state_manager_instance.check_message_db(sampleRobotCode, rosmsg);

//   // Check if log is created
//   log_id++;
//   std::string filename = log_name + std::to_string(log_id) + log_ext;

//   // Check if file exists
//   std::ifstream infile1(filename);
//   bool fileflag = infile1.good();
//   ASSERT_TRUE(fileflag);

//   // Call check_message_db again with the same message
//   state_manager_instance.check_message_db(sampleRobotCode, rosmsg);

//   // Check if log is created
//   log_id++;
//   filename = log_name + std::to_string(log_id) + log_ext;

//   // Check if file exists
//   std::ifstream infile2(filename);
//   fileflag = infile2.good();
//   ASSERT_TRUE(fileflag);

//   // Clear state manager
//   state_manager_instance.clear();
// }

int main(int argc, char **argv)
{
  // Set log folder
  setLogFolder();

  // Start tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
