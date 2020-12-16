import os
import time
import unittest
import json

import launch
import launch_ros
import launch_ros.actions
import launch_testing.util
import launch_testing_ros
import rclpy
import rclpy.context
import rclpy.executors
import rcl_interfaces.msg


LOGID = 0
os.environ['AGENT_TYPE'] = 'ROS'
os.environ['AGENT_MODE'] = 'JSON_TEST'

def generate_test_description(ready_fn):
    # Necessary to get real-time stdout from python processes but we are not using it:
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    listener_node = launch_ros.actions.Node(
        package='error_resolution_diagnoser_ros2',
        node_executable='error_resolution_diagnoser_ros2',
        arguments=['__log_disable_rosout:=true'],
        env=proc_env,
    )

    return (
        launch.LaunchDescription([
            listener_node,
            # Start tests right away - no need to wait for anything
            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ]),
        {
            'listener': listener_node,
        }
    )


class ListenerTest(unittest.TestCase):

    # Method to setup properties of the test
    @classmethod
    def setUpClass(cls, proc_output, listener):

        # Test node
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('test_node', context=cls.context)
        
        # Log file info
        homevar = os.path.expanduser("~")
        parentdir = homevar + "/.cognicept/agent/logs/latest_log_loc.txt"
        f = open(parentdir)
        latest_log_loc = f.readline()
        latest_log_loc = latest_log_loc[:-1]
        latest_log_loc = latest_log_loc.replace("/$HOME", homevar)
        print("Reading logs from: ", latest_log_loc)
        f.close()
        cls.logfolder = latest_log_loc
        # else:
            # self.logfolder = "/app/logs"
        
        cls.logname = cls.logfolder + "/logData"
        cls.logext = ".json"
        cls.logid = 0

    # Method to spin ROS2 thread once
    def spin_rclpy(self, timeout_sec):
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)
        try:
            executor.spin_once(timeout_sec=timeout_sec)
        finally:
            executor.remove_node(self.node)

    # Method to publish messages
    def talk(self, msg_list, sev_list, pub, msg, listener):
        print('Talking...')

        # Loop through message list    
        for idx in range(len(msg_list)):
            
            # Wait for process response
            success = self.proc_output.waitFor(
                expected_output=msg.msg,
                process=listener,
                timeout=1.0,
            )

            # Set message content
            msg.msg = msg_list[idx]
            print(msg_list[idx])

            # Set message severity            
            if sev_list[idx] == "E":
                msg.level = 40
            elif sev_list[idx] == "W":
                msg.level = 30
            else:
                msg.level = 20
            
            # Publish message
            pub.publish(msg)

            # Spin away
            self.spin_rclpy(0.25)

    """
    TEST CASES:
    CASE 1: Error Suppression
    CASE 2: Info Suppression
    CASE 3: Warn Suppression
    """

    def test_error_suppression_log(self, listener):

        # Create publisher
        pub = self.node.create_publisher(
            rcl_interfaces.msg.Log,
            'rosout',
            10
        )
        self.addCleanup(self.node.destroy_publisher, pub)

        # Create a sample message
        msg = rcl_interfaces.msg.Log()
        msg.msg = "Sample message to establish plumbing?"
        msg.level = 40
        
        # Publish a sample message to establish plumbing for the first time (only for Dashing)
        if(os.environ['ROS_DISTRO'] == 'dashing'):
            pub.publish(msg)
        
        # Test message list
        msg_list = [
            "Begin navigating from current location to (-1.65, 1.28)",
            "Planning algorithm failed to generate a valid path to (-2.02, -1.21)",
            "Attempting Spin",
            "Turning -1.57 for spin recovery",
            "Spin running...",
            "Spin running...",
            "Spin running...",
            "Spin completed successfully",
            "Planning algorithm failed to generate a valid path to (-2.02, -1.21)",
            "Navigation failed",
            "Goal was canceled. Stopping the robot.",
            "Begin navigating from current location to (-1.65, 1.28)",
            "Planning algorithm failed to generate a valid path to (-2.02, -1.21)",
            "Attempting Spin",
            "Turning -1.57 for spin recovery",
            "Spin running...",
            "Spin running...",
            "Spin completed successfully",
            "Planning algorithm failed to generate a valid path to (-2.02, -1.21)",
            "Navigation failed",
            "Goal was canceled. Stopping the robot."
            ]

        # Test severity level that should match with message list
        sev_list = ["I", "W", "I", "I", "I", "I", "I", "I", "W", "E", "I",
                    "I", "W", "I", "I", "I", "I", "I", "W", "E", "I"]

        # Send messages
        self.talk(msg_list, sev_list, pub, msg, listener)

        # Check if logs are correctly created

        global LOGID

        # Set up test variables
        expected_logs = 16        
        log_idx = 0
        message = []
        event_id = []
        create_ticket = []
        
        # Loop through logs
        while(log_idx < expected_logs):
            # Check if logs are created
            log_idx += 1
            LOGID += 1
            filename = self.logname + str(LOGID) + self.logext
            print("Checking: ", filename)
            file_flag = os.path.isfile(filename)
            if file_flag is False:
                LOGID = 16
                break

            # Retrieve values
            with open(filename) as json_file:
                data = json.load(json_file)
                message.append(data['message'])
                event_id.append(data['event_id'])
                create_ticket.append(data['create_ticket'])

        # Check if expected log files are created        
        self.assertTrue(file_flag)

        # Check if messages are expected
        expected_msg_list = [
            "Begin navigating from current location to (-1.65, 1.28)",
            "Planning algorithm failed to generate a valid path to (-2.02, -1.21)",
            "Attempting Spin",
            "Turning -1.57 for spin recovery",
            "Spin running...",
            "Spin completed successfully",
            "Navigation failed",
            "Goal was canceled. Stopping the robot.",
            "Begin navigating from current location to (-1.65, 1.28)",
            "Planning algorithm failed to generate a valid path to (-2.02, -1.21)",
            "Attempting Spin",
            "Turning -1.57 for spin recovery",
            "Spin running...",
            "Spin completed successfully",
            "Navigation failed",
            "Goal was canceled. Stopping the robot."]
        
        self.assertEqual(expected_msg_list, message)

        # Check if create_ticket is correct
        expected_create_ticket = [False, False, False, False, False, False, True, False,
                                  False, False, False, False, False, False, True, False]
        self.assertEqual(expected_create_ticket, create_ticket)

        # Check if event_ids are unique
        expected_events = 3
        unique_event_flag = (expected_events == len(set(event_id)))
        self.assertTrue(unique_event_flag)

    def test_info_suppression_log(self, listener):
            
        # Create publisher
        pub = self.node.create_publisher(
            rcl_interfaces.msg.Log,
            'rosout',
            10
        )
        self.addCleanup(self.node.destroy_publisher, pub)

        # Create message
        msg = rcl_interfaces.msg.Log()
        
        # Test message list
        msg_list = [
            "initialPoseReceived",
            "Setting pose (375.252000): -1.564 1.400 0.786",
            "Begin navigating from current location to (-1.65, 1.28)",
            "Received a goal, begin following path",
            "Preempting the goal. Passing the new path to the planner.",
            "Preempting the goal. Passing the new path to the planner.",
            "Preempting the goal. Passing the new path to the planner.",
            "Preempting the goal. Passing the new path to the planner.",
            "Preempting the goal. Passing the new path to the planner.",
            "Reached the goal!",
            "Navigation succeeded"]

        # Test severity level that should match with message list
        sev_list = ["I", "I", "I", "I", "I", "I", "I", "I", "I", "I", "I"]

        # Send messages
        self.talk(msg_list, sev_list, pub, msg, listener)

        # Check if logs are correctly created

        global LOGID

        # Set up test variables
        expected_logs = 7        
        log_idx = 0
        message = []
        event_id = []
        create_ticket = []
        
        # Loop through logs
        while(log_idx < expected_logs):
            # Check if logs are created
            log_idx += 1
            LOGID += 1
            filename = self.logname + str(LOGID) + self.logext
            file_flag = os.path.isfile(filename)
            
            if file_flag is False:
                LOGID = 23
                break

            # Retrieve values
            with open(filename) as json_file:
                print(filename)
                data = json.load(json_file)
                message.append(data['message'])
                event_id.append(data['event_id'])
                create_ticket.append(data['create_ticket'])

        # Check if expected log files are created
        self.assertTrue(file_flag)

        # Check if messages are expected
        expected_msg_list = [
            "initialPoseReceived",
            "Setting pose (375.252000): -1.564 1.400 0.786",
            "Begin navigating from current location to (-1.65, 1.28)",
            "Received a goal, begin following path",
            "Preempting the goal. Passing the new path to the planner.",
            "Reached the goal!",
            "Navigation succeeded"]
        
        self.assertEqual(expected_msg_list, message)

        # Check if create_ticket is correct
        expected_create_ticket = [False, False, False, False, False, False, False]
        self.assertEqual(expected_create_ticket, create_ticket)

        # Check if event_ids are unique
        expected_events = 1
        unique_event_flag = (expected_events == len(set(event_id)))
        self.assertTrue(unique_event_flag)

    def test_warn_suppression_log(self, listener):
        
        # Create publisher                
        pub = self.node.create_publisher(
            rcl_interfaces.msg.Log,
            'rosout',
            10
        )
        self.addCleanup(self.node.destroy_publisher, pub)
        
        # Create message
        msg = rcl_interfaces.msg.Log()
        
        # Test message list
        msg_list = [
            "Begin navigating from current location to (-1.65, 1.28)",
            "Planning algorithm failed to generate a valid path to (-2.02, -1.21)",
            "Attempting Spin",
            "Turning -1.57 for spin recovery",
            "Spin running...",
            "Spin completed successfully",
            "Planning algorithm failed to generate a valid path to (-2.02, -1.21)",
            "Navigation failed"
            ]

        # Test severity level that should match with message list
        sev_list = ["I", "W", "I", "I", "I", "I", "W", "E"]

        # Send messages
        self.talk(msg_list, sev_list, pub, msg, listener)

        # Check if logs are correctly created

        global LOGID

        # Set up test variables
        expected_logs = 7        
        log_idx = 0
        message = []
        event_id = []
        create_ticket = []
        
        # Loop through logs
        while(log_idx < expected_logs):
            # Check if logs are created
            log_idx += 1
            LOGID += 1
            filename = self.logname + str(LOGID) + self.logext
            file_flag = os.path.isfile(filename)
            
            if file_flag is False:
                LOGID = 30
                break

            # Retrieve values
            with open(filename) as json_file:
                print(filename)
                data = json.load(json_file)
                message.append(data['message'])
                event_id.append(data['event_id'])
                create_ticket.append(data['create_ticket'])
        
        LOGID += expected_logs

        # Check if expected log files are created
        self.assertTrue(file_flag)

        # Check if messages are expected
        expected_msg_list = [
            "Begin navigating from current location to (-1.65, 1.28)",
            "Planning algorithm failed to generate a valid path to (-2.02, -1.21)",
            "Attempting Spin",
            "Turning -1.57 for spin recovery",
            "Spin running...",
            "Spin completed successfully",
            "Navigation failed"]
        
        self.assertEqual(expected_msg_list, message)

        # Check if create_ticket is correct
        expected_create_ticket = [False, False, False, False, False, False, True]
        self.assertEqual(expected_create_ticket, create_ticket)

        # Check if event_ids are unique
        expected_events = 1
        unique_event_flag = (expected_events == len(set(event_id)))
        self.assertTrue(unique_event_flag)
