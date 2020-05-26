# rosrect Listener Agent Documentation 

This project adheres to the Contributor Covenant [code of conduct](https://github.com/cognicept-admin/rosrect/blob/master/CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to [info@cognicept.systems](mailto:info@cognicept.systems). If you are interested in contributing, please refer to the guidelines [here](https://github.com/cognicept-admin/rosrect/blob/master/CONTRIBUTING.md).  

- [Description](#description)
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running tests](#running-tests)
- [Syntax](#syntax)
- [Example-Application](#example-application)
    * [Catching Navigation Errors from /move_base](catching-navigation-errors-from-/move_base)
    * [Start Simulation](#start-simulation)
    * [Start rosrect Listener Agent](#start-rosrect-listener-agent)
    * [Generate a navigation error](#generate-a-navigation-error)
- [Related-Pages](#related-pages)

## Description
This article explains how to run the `rosrect Listener Agent` ROS node for ROS 2.

## Overview
This article shows how to start the `rosrect Listener Agent`. By the end of this, you will be able to start the agent, run a simulation and test the listener agent to listen to navigation errors.

## Prerequisites
Some knowledge of ROS 2 and robotics is necessary.

## Installation

You can get access to the agent by cloning this repo and building the ROS node. Steps are as follows:

1. Open a terminal window.

2. Install Microsoft's [`C++ REST SDK`][6] for establishing the backend api for incident management using `apt-get`:

    ```
    $ sudo apt-get install libcpprest-dev
    ```

2. Change to your `src` folder of the ROS 2 workspace directory. Generally it is as follows:

    ```
    $ cd ~/ros2_ws/src
    ```

3. Clone the repo:

    ```git
    $ git clone https://github.com/cognicept-admin/rosrect-listener-agent-ros2
    ```

4. Change to your `ros2_ws` folder:

    ```
    $ cd ..
    ``` 

5. Issue `colcon build` to build the ROS node:

    ```
    $ colcon build --symlink-install
    ```

6. Check if node has built correctly and registered using `ros2 pkg`:

    ```
    $ ros2 pkg list | grep rosrect
      rosrect-listener-agent-ros2
    ```

That is it for the installation!

## Running tests
Optionally, you can run the unit tests by following steps below. 

1. Open a new terminal and switch to the `ros2_ws` directory:

    ```
    $ cd ~/ros2_ws
    ```

2. Run tests using `colcon test`. Your terminal will show test results similar to a snapshot below. Logs will be created in the `./test/logs` folder:

    ```

    $ colcon test --packages-select rosrect-listener-agent-ros2
    Starting >>> rosrect-listener-agent
    Finished <<< rosrect-listener-agent [25.3s]            

    Summary: 1 package finished [25.5s]

    ```

3. See the test results using `colcon test-result` as follows. Use `--verbose` option to see the test failure results, and `-all` to see all results.
    
    The following shows that no tests failed:

    ```

    $ colcon test-result --verbose
    Summary: 24 tests, 0 errors, 0 failures, 0 skipped

    ```

    The following shows the results no matter whether it passed or failed. Open up the individual XML files for more information:

    ```

    $ colcon test-result --all
    build/rosrect-listener-agent/Testing/20200526-0218/Test.xml: 4 tests, 0 errors, 0 failures, 0 skipped
    build/rosrect-listener-agent/test_results/rosrect-listener-agent/backend_test_node.gtest.xml: 2 tests, 0 errors, 0 failures, 0 skipped
    build/rosrect-listener-agent/test_results/rosrect-listener-agent/robotevent_test_node.gtest.xml: 4 tests, 0 errors, 0 failures, 0 skipped
    build/rosrect-listener-agent/test_results/rosrect-listener-agent/statemanager_test_node.gtest.xml: 8 tests, 0 errors, 0 failures, 0 skipped
    build/rosrect-listener-agent/test_results/rosrect-listener-agent/test_listener_integration_launch_ros_test.py.xunit.xml: 3 tests, 0 errors, 0 failures, 0 skipped
    build/rosrect-listener-agent/test_results/rosrect-listener-agent/test_listener_integration_launch_test.py.xunit.xml: 3 tests, 0 errors, 0 failures, 0 skipped

    Summary: 24 tests, 0 errors, 0 failures, 0 skipped
    
    ```

## Syntax
The listener agent is expecting some environment variables to be set as follows:

* `ROBOT_CODE` - A unique code that identifies the robot the agent is listening to. Usually a UUID but any string will work.
* `SITE_CODE` - A unique code that identifies the site of the robot being listened to. Usually a UUID but any string will work.
* `AGENT_ID` - A unique code that identifies the agent that is listening. Usually a UUID but any string will work.
* `AGENT_MODE` - A variable that when set to value TEST, will save JSON logs as "outputs" of the listener in the logs folder.
* `AGENT_TYPE` - The same listener can be operated to catch *ANY* ROS log or logs that are only available as part of the Error Classification System (ECS) to enable log suppression for particular robots/sites. Set it to value ROS or DB respectively. **NOTE:** The ECS feature is currently under development and is unavailable for use. 

For example,
```
$ export ROBOT_CODE=R2D2
$ export SITE_CODE=MFALCON
$ export AGENT_ID=DROID
$ export AGENT_MODE=TEST
$ export AGENT_TYPE=ROS
```
**Note: These values are available only in the current terminal and need to be recreated every time before running the listener. One way to get around this is to place these statements in the `bashrc` file**

Now, you can run the listener agent using the provided launch file and `ros2 launch`:
```
$ ros2 launch rosrect-listener-agent listener-agent-launch.py
```
**This is just a syntax. We will be using this listener agent to connect to a simulation to listen to errors in the next section!**

## Example Application

### Catching Navigation Errors from Navigation 2
In this example, we will run the `rosrect Listener Agent` along with the Turtlebot3 simulation to see how ROS 2's [Navigation 2][4] errors are caught.

### Start Simulation
Start a Turtlebot3 Navigation demo as documented [here][5]. Please make sure you have gone through the above documentation if you are facing any errors with this step.

Open a new terminal and launch the `nav2_simulation_launch` launch file:

```
$ export TURTLEBOT3_MODEL=waffle
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
$ ros2 launch nav2_bringup nav2_simulation_launch.py
```

**Note: This will open an `RViz2` window. This could be an empty window and needs to be configured. Navigation 2 requires specific configuration as follows.**

In the empty RViz2 window, click on **Startup** button in the bottom left area.

![alt text](/docs/images/startupButton.png "Navigation Startup")

You should now see a map as shown below. To see the sensor information, localize the robot by using the **2D Pose Estimate** button:

![alt text](/docs/images/poseEstimateButton.png "Pose Estimate")

For the purposes of the demonstration, make sure that the robot is intentionally mislocalized. (i.e. the scan doesn't match the map) as shown below:

![alt text](/docs/images/Mislocalized.png "Turtlebot mislocalized")

### Start rosrect Listener Agent
We are ready to start listening to robot errors. Simply launch the listener ROS 2 node using the launch file:

```
$ ros2 launch rosrect-listener-agent-ros2 listener-agent-launch.py
[INFO] [launch]: All log files can be found below /home/swaroophs/.ros/log/2020-05-26-11-24-37-178020-swarooph-xps-22744
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rosrect-listener-agent-1]: process started with pid [22772]
[rosrect-listener-agent-1] TEST mode is ON. JSON Logs will be saved here: /home/swaroophs/ros2_dd_ws/src/rosrect-listener-agent/test/logs/
[rosrect-listener-agent-1] Subscribed to Listener Agent with direct rosout...

```

### Generate a navigation error
Now, use `Rviz2` to provide a `Navigation2 goal` for the robot. 

![alt text](/docs/images/NavGoal.png "Navigation2 Goal in Rviz2")

Because the robot is mislocalized, chances are high that it will be unable to reach its goal, generating an error. When that happens, the terminal window running the simulation will show something like the following:
```
1590486170.4089003 [bt_navigator-10] [INFO] [bt_navigator]: Begin navigating from current location to (-2.04, -1.17)
1590486170.4169328 [navfn_planner-9] [WARN] [navfn_planner]: Planning algorithm failed to generate a valid path to (-2.04, -1.17)
1590486171.4250529 [recoveries_node-11] [INFO] [recoveries]: Attempting Spin
1590486171.4261382 [recoveries_node-11] [INFO] [recoveries]: Turning -1.57 for spin recovery.
1590486172.4256592 [recoveries_node-11] [INFO] [recoveries]: Spin running...
1590486173.4258511 [recoveries_node-11] [INFO] [recoveries]: Spin running...
1590486173.5255470 [recoveries_node-11] [INFO] [recoveries]: Spin completed successfully
1590486173.5320866 [navfn_planner-9] [WARN] [navfn_planner]: Planning algorithm failed to generate a valid path to (-2.04, -1.17)
1590486173.5508025 [recoveries_node-11] [INFO] [recoveries]: Attempting Spin
1590486173.5510445 [recoveries_node-11] [INFO] [recoveries]: Turning -1.57 for spin recovery.
1590486174.5470121 [recoveries_node-11] [INFO] [recoveries]: Spin running...
1590486175.4535339 [recoveries_node-11] [INFO] [recoveries]: Spin completed successfully
1590486175.4593930 [navfn_planner-9] [WARN] [navfn_planner]: Planning algorithm failed to generate a valid path to (-2.04, -1.17)
1590486175.4745233 [bt_navigator-10] [ERROR] [bt_navigator]: Navigation failed
```

The terminal window running the listener agent will show the following. Depending on the number of errors in the previous window, you will see the same number of JSON logs generated:

```
[rosrect-listener-agent-1] Message received: Begin navigating from current location to (-2.04, -1.17)
[rosrect-listener-agent-1] Info Event logged with id: 9011c49b-c746-4453-aaa9-d823c352d84c
[rosrect-listener-agent-1] /home/swaroophs/ros2_dd_ws/src/rosrect-listener-agent/test/logs/logData1.json
[rosrect-listener-agent-1] Message received: Planning algorithm failed to generate a valid path to (-2.04, -1.17)
[rosrect-listener-agent-1] Warning Event logged with id: 9011c49b-c746-4453-aaa9-d823c352d84c
[rosrect-listener-agent-1] /home/swaroophs/ros2_dd_ws/src/rosrect-listener-agent/test/logs/logData2.json
[rosrect-listener-agent-1] Message received: Attempting Spin
[rosrect-listener-agent-1] Info Event logged with id: 9011c49b-c746-4453-aaa9-d823c352d84c
[rosrect-listener-agent-1] /home/swaroophs/ros2_dd_ws/src/rosrect-listener-agent/test/logs/logData3.json
[rosrect-listener-agent-1] Message received: Turning -1.57 for spin recovery.
[rosrect-listener-agent-1] Info Event logged with id: 9011c49b-c746-4453-aaa9-d823c352d84c
[rosrect-listener-agent-1] /home/swaroophs/ros2_dd_ws/src/rosrect-listener-agent/test/logs/logData4.json
[rosrect-listener-agent-1] Message received: Spin running...
[rosrect-listener-agent-1] Info Event logged with id: 9011c49b-c746-4453-aaa9-d823c352d84c
[rosrect-listener-agent-1] /home/swaroophs/ros2_dd_ws/src/rosrect-listener-agent/test/logs/logData5.json
[rosrect-listener-agent-1] Message received: Spin running...
[rosrect-listener-agent-1] Message received: Spin completed successfully
[rosrect-listener-agent-1] Info Event logged with id: 9011c49b-c746-4453-aaa9-d823c352d84c
[rosrect-listener-agent-1] /home/swaroophs/ros2_dd_ws/src/rosrect-listener-agent/test/logs/logData6.json
[rosrect-listener-agent-1] Message received: Planning algorithm failed to generate a valid path to (-2.04, -1.17)
[rosrect-listener-agent-1] Message received: Attempting Spin
[rosrect-listener-agent-1] Message received: Turning -1.57 for spin recovery.
[rosrect-listener-agent-1] Message received: Spin running...
[rosrect-listener-agent-1] Message received: Spin completed successfully
[rosrect-listener-agent-1] Message received: Planning algorithm failed to generate a valid path to (-2.04, -1.17)
[rosrect-listener-agent-1] Message received: Navigation failed
[rosrect-listener-agent-1] Error Event logged with id: 9011c49b-c746-4453-aaa9-d823c352d84c
[rosrect-listener-agent-1] /home/swaroophs/ros2_dd_ws/src/rosrect-listener-agent/test/logs/logData7.json
```

Let's look at an example JSON log, `logData7.json`. Note the `agent_id`, `robot_id` and `property_id` are same as the environment variables set before. `message` has information about the `rosout` actual message and a unique UUID `event_id`. This also has a flag `create_ticket` that can be used by downstream systems to trigger particular actions such as creating tickets or notifications:
```JSON
{
    "agent_id": "DROID",
    "compounding": "Null",
    "create_ticket": true,
    "description": "Null",
    "event_id": "3257342f-f199-4a77-96f9-3042a8f50119",
    "level": "Error",
    "message": "Navigation failed",
    "module": "Null",
    "property_id": "MFALCON",
    "resolution": "Null",
    "robot_id": "R2D2",
    "source": "bt_navigator",
    "timestamp": "2020-05-26T11:24:59.128648"
}
```
This shows that the listener agent is able to successfully capture `rosout` logs and report it as a JSON structure. These JSON structures are created whenever there is `rosout` message. It associates an `event_id` for each of the messages. This unique `event_id` is reset whenever a log with [severity level][3] `ERROR` or `Goal reached` message. This means multiple JSON logs can be combined into a single **event log** using these `event_id`s. 

**NOTE:** You will not see a log for EACH of the `rosout` message seen on screen. Here are some scenarios to consider:

* Screen doesn't always show all `rosout` messages. Some nodes are publishing directly to the topic for the log and not displaying it on screen. These messages will also create logs. E.g. Setting goals/poses.
* During the same *event* duplicate messages will *NOT* create logs. This suppression logic is intentional and built into the listener via a `State Manager`. E.g. the log above shows 3 "Got new plan" messages. However only one log will be created for this. At the end of this *event*, there is an error, which will trigger the end of the *event*. So any other "Got new plan" messages in the future WILL create a log.

These JSON logs can be consumed by REST APIs/data streams to connect to incident management/monitoring systems to keep track of robot errors. Now, operators can monitor this incident management system to intervene robot operations to correct the errors to reduce downtime on the actual field.

## Related Pages
For more related information, refer to:

* [Getting Started with Navigation 2][1]
* [Turtlebot3 installation][2]
* [ROS 2 log message structure][3]
* [Microsoft C++ REST SDK][6]

[1]: https://navigation.ros.org/getting_started/index.html
[2]: https://navigation.ros.org/getting_started/index.html#installation
[3]: https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/Log.msg
[4]: https://navigation.ros.org/index.html
[5]: https://navigation.ros.org/getting_started/index.html#
[6]: https://github.com/microsoft/cpprestsdk