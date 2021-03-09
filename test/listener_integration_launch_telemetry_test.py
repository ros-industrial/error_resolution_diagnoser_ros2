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
import nav_msgs.msg
import geometry_msgs.msg
from datetime import datetime
from dateutil.parser import *

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

        cls.logname = cls.logfolder + "/logDataStatus"
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
    def talk(self, odom_pub, odom_msg, pose_pub, pose_msg, listener):
        print("Publishing sample telemetry information...")

        # Set static values for messages 
        odom_msg.pose.pose.position.x = 1.0
        odom_msg.pose.pose.position.y = 1.0
        odom_msg.pose.pose.position.z = 1.0
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.pose.pose.orientation.x = 1.0
        odom_msg.pose.pose.orientation.y = 1.0
        odom_msg.pose.pose.orientation.z = 1.0

        pose_msg.pose.pose.position.x = 2.0
        pose_msg.pose.pose.position.y = 2.0
        pose_msg.pose.pose.position.z = 2.0
        pose_msg.pose.pose.orientation.w = 2.0
        pose_msg.pose.pose.orientation.x = 2.0
        pose_msg.pose.pose.orientation.y = 2.0
        pose_msg.pose.pose.orientation.z = 2.0

        # Loop through messages
        for idx in range(20):

            # Wait for process response
            success = self.proc_output.waitFor(
                expected_output="None",
                process=listener,
                timeout=1.0,
            )

            # Set dynamic message values
            odom_msg.pose.pose.position.x = float(idx)
            odom_msg.pose.pose.orientation.x = float(idx)
            # print('Odom msg: ', odom_msg)
            pose_msg.pose.pose.position.x = float(idx)
            pose_msg.pose.pose.orientation.x = float(idx)
            # print('Pose msg: ', pose_msg)

            # Publish messages
            odom_pub.publish(odom_msg)
            pose_pub.publish(pose_msg)

            # Spin away
            self.spin_rclpy(0.25)

    """
    TEST CASES:
    CASE 1: Telemetry test
    """

    def test_telemetry(self, listener):

        # Go through the lifecycle of a heartbeat to test telemetry

        # When agent node starts, heartbeat is generated and includes no telemetry 
        # since those topics have not started publishing
        print("Starting pre-telemetry checks...")
        # Check if logs are created
        filename = self.logname + self.logext
        print("Checking: ", filename)
        file_flag = os.path.isfile(filename)

        # Check if expected log files are created
        self.assertTrue(file_flag)

        # Set up test variables
        message = []
        create_ticket = []
        telemetry = []
        timestamp1 = []

        # Retrieve values
        with open(filename) as json_file:
            data = json.load(json_file)
            # print(data)
            message = data['message']
            create_ticket = data['create_ticket']
            telemetry = data['telemetry']
            timestamp1 = parse(data['timestamp'])

        # Check if message is correct
        self.assertEqual('Online', message)

        # Check if create_ticket is correctly set to False
        self.assertEqual(False, create_ticket)

        # Check if telemetry is empty
        self.assertEqual(None, telemetry)

        # Start publishing telemetry topics and check again.
        # This is also a good time to check timestamps.
        print("Starting telemetry checks. This will take around 20 seconds...")
        
        # Create publishers
        odom_pub = self.node.create_publisher(
            nav_msgs.msg.Odometry,
            'odom',
            10
        )
        pose_pub = self.node.create_publisher(
            geometry_msgs.msg.PoseWithCovarianceStamped,
            'amcl_pose',
            10
        )
        self.addCleanup(self.node.destroy_publisher, odom_pub)

        # Create a sample message
        odom_msg = nav_msgs.msg.Odometry()
        pose_msg = geometry_msgs.msg.PoseWithCovarianceStamped()

        # Send messages for at least 15 seconds so new heartbeat is guaranteed to generate.
        # This will include the telemetry info.
        self.talk(odom_pub, odom_msg, pose_pub, pose_msg, listener)

        # Check if logs are created
        filename = self.logname + self.logext
        print("Checking: ", filename)
        file_flag = os.path.isfile(filename)

        # Check if expected log files are created
        self.assertTrue(file_flag)

        # Set up test variables
        message = []
        create_ticket = []
        telemetry = []
        timestamp2 = []

        # Retrieve values
        with open(filename) as json_file:
            data = json.load(json_file)
            # print(data)
            message = data['message']
            create_ticket = data['create_ticket']
            telemetry = data['telemetry']
            timestamp2 = parse(data['timestamp'])

        # Check if message is correct
        self.assertEqual('Online', message)

        # Check if create_ticket is correctly set to False
        self.assertEqual(False, create_ticket)

        # Check if telemetry is has the right keys
        keys = list(telemetry.keys())
        expected_keys = ['nav_pose', 'odom_pose']
        self.assertEqual(expected_keys, keys)

        # Check if timestamps are at least 15 seconds apart
        duration = timestamp2 - timestamp1
        duration_in_s = duration.total_seconds()
        self.assertLess(14, duration_in_s)
