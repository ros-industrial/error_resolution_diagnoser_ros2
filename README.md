# error_resolution_diagnoser_ros2 Documentation 

[![Build Status](https://jenkins.cognicept.systems/buildStatus/icon?job=cognicept-ros2-agent-pipeline)](https://jenkins.cognicept.systems/job/cognicept-ros2-agent-pipeline/)  [![Coverage Status](http://34.87.159.179:5000/coverage/cognicept-ros2-agent-pipeline)](http://34.87.159.179:5000/coverage/cognicept-ros2-agent-pipeline)

Hello there! Thanks for checking out the documentation. This particular document is a user's guide. If you are more interested in what the `error_resolution_diagnoser_ros2` is designed for, and the architecture, please take a look at the introduction document [here][7]!

This project adheres to the Contributor Covenant [code of conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to [info@cognicept.systems](mailto:info@cognicept.systems). If you are interested in contributing, please refer to the guidelines [here](CONTRIBUTING.md).  

- [Description](#description)
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
    * [Building Natively](#building-natively)
    * [Building through Docker](#building-through-docker)
- [Running tests](#running-tests)
    * [Native](#native)
    * [Using Docker](#using-docker)
- [Syntax](#syntax)
    * [Configure and Run for native installations](#configure-and-run-for-native-installations)
    * [Configure and Run for Docker](#configure-and-run-for-docker)
- [Example-Application](#example-application)
    * [Catching Navigation Errors from Navigation 2](#catching-navigation-errors-from-navigation-2)
    * [Start Simulation](#start-simulation)
    * [Start error_resolution_diagnoser_ros2](#start-error_resolution_diagnoser_ros2)
    * [Generate a navigation error](#generate-a-navigation-error)
- [Related-Pages](#related-pages)

## Description
This article explains how to run the `error_resolution_diagnoser_ros2` ROS node for ROS 2. For the rest of the documentation, the term `agent` will be used as a shorthand to refer to the `error_resolution_diagnoser_ros2`.

## Overview
This article shows how to start the `error_resolution_diagnoser_ros2`. By the end of this, you will be able to start the agent, run a simulation and test the listener agent to listen to navigation errors.

## Prerequisites
Some knowledge of ROS 2 and robotics is necessary.

## Installation

You can get access to the agent by cloning this repo. After this, there are a couple of choices as to how you want to build and run the agent ROS node. Either natively, or using Docker. Steps are as follows:

1. Open a terminal window.

2. Change to your `src` folder of the ROS 2 workspace directory. Generally it is as follows:

        $ cd ~/ros2_ws/src
    
3. Clone the repo:

        $ git clone https://github.com/cognicept-admin/error_resolution_diagnoser_ros2
    
### Building natively:

You can use this approach if you are planning on running this on a system that has a working ROS 2 installation. Steps are as follows:

1. Install Microsoft's [`C++ REST SDK`][5] for establishing the backend api for incident management using `apt-get`:

        $ sudo apt-get install libcpprest-dev
    
2. Install ROS 2's launch testing framework if it is not installed already using `apt-get`. Here `< ROSDISTRO >` stands for your ROS 2 distribution such as `dashing` or `eloquent`:
    
        $ sudo apt-get install ros-< ROSDISTRO >-launch-testing*
    
3. Change to your `ros2_ws` folder:

        $ cd ..
     
4. Issue `colcon build` to build the ROS node :

        $ colcon build --symlink-install
    
5. Source the changes by running the `setup.bash` file:

        $ source ~/ros2_ws/install/setup.bash
    
6. Check if node has built correctly and registered using `ros2 pkg`:

        $ ros2 pkg list | grep error_resolution_diagnoser_ros2
        error_resolution_diagnoser_ros2

7. Additionally, follow the appropriate installation steps for installing the `ECS API Server` [here][8].
    
That is it for the native installation! You can now jump to [Running tests](#running-tests) or [Syntax](#syntax).

### Building through Docker:

You can use this approach if you are planning on running the agent on a system that does not have ROS but will be connected to the same ROS network. 

1. First, make sure you have a working [Docker installation][6].

2. You can then build the `docker` image using `docker build` and the provided `Dockerfile`:

        $ docker build -t error_resolution_diagnoser_ros2 .

3. Additionally, follow the appropriate installation steps for installing the `ECS API Server` [here][8].
    
That is it for the Docker installation! You can now jump to [Running tests](#running-tests) or [Syntax](#syntax).

## Running tests
Optionally, you can run the unit and integration tests natively or using Docker, based on the installation method you chose in the previous section.

**NOTE: Before running tests, makes sure the `ECS API Server` is running either natively or using Docker. Take a look at the relevant documentation [here][9]. Failure to have the API server will result in some failed tests that require connection to ECS.**

### Native

1. Open a new terminal and switch to the `ros2_ws` directory:

        $ cd ~/ros2_ws
    
2. Run tests using `colcon test` as shown below. Logs will be created in the `/$HOME/.cognicept/agent/logs` folder:

        $ colcon test --packages-select error_resolution_diagnoser_ros2
        Starting >>> error_resolution_diagnoser_ros2
        Finished <<< error_resolution_diagnoser_ros2 [24.0s]             

        Summary: 1 package finished [25.5s]
    
3. See the test results using `colcon test-result` as follows. Use `--verbose` option to see the test failure results, and `-all` to see all results.
    
    The following shows that no tests failed:

        $ colcon test-result --verbose
        Summary: 37 tests, 0 errors, 0 failures, 0 skipped

    The following shows the results no matter whether it passed or failed. Open up the individual XML files for more information:

    
        $ colcon test-result --all
        build/error_resolution_diagnoser_ros2/Testing/20200708-0252/Test.xml: 5 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/backend_test_node.gtest.xml: 6 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/robotevent_test_node.gtest.xml: 5 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/statemanager_test_node.gtest.xml: 8 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/test_listener_integration_launch_db_test.py.xunit.xml: 5 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/test_listener_integration_launch_ros_test.py.xunit.xml: 3 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/Testing/20200617-0728/Test.xml: 5 tests, 0 errors, 0 failures, 0 skipped

        Summary: 37 tests, 0 errors, 0 failures, 0 skipped
    

### Using Docker

1. Make sure that you have built the docker image by following the steps [here](#building-through-docker).

2. Switch to the repository's folder or wherever you might be storing the `runtime.env` file.

        $ cd ~/ros2_ws/src/error_resolution_diagnoser_ros2
    
3. You can run the tests by using the following `docker run` command. This will run the `buildntest.bash` script to run the tests and pull the results. Logs will be created in the `/$HOME/.cognicept/agent/logs` folder:

        $ docker run -it \
        --env-file runtime.env \
        --network=host \
        --name=agent  \
        --volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
        error_resolution_diagnoser_ros2:latest  \
        /home/ros2_ws/src/error_resolution_diagnoser_ros2/buildntest.bash
        
        STEP 1: Building agent code...
        Starting >>> error_resolution_diagnoser_ros2
        Finished <<< error_resolution_diagnoser_ros2 [0.38s]                       

        Summary: 1 package finished [0.55s]
        /home/ros2_ws/src/error_resolution_diagnoser_ros2/buildntest.bash: line 4: /root/ros2_ws/install/setup.bash: No such file or directory
        STEP 2: Testing agent code...
        Starting >>> error_resolution_diagnoser_ros2
        [Processing: error_resolution_diagnoser_ros2]                   
        Finished <<< error_resolution_diagnoser_ros2 [36.8s]            

        Summary: 1 package finished [37.0s]
        STEP 3: Getting failures...
        Summary: 32 tests, 0 errors, 0 failures, 0 skipped
        STEP 4: Getting all test results...
        build/error_resolution_diagnoser_ros2/Testing/20200708-0630/Test.xml: 5 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/backend_test_node.gtest.xml: 6 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/robotevent_test_node.gtest.xml: 5 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/statemanager_test_node.gtest.xml: 8 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/test_listener_integration_launch_db_test.py.xunit.xml: 5 tests, 0 errors, 0 failures, 0 skipped
        build/error_resolution_diagnoser_ros2/test_results/error_resolution_diagnoser_ros2/test_listener_integration_launch_ros_test.py.xunit.xml: 3 tests, 0 errors, 0 failures, 0 skipped

        Summary: 32 tests, 0 errors, 0 failures, 0 skipped
    
## Syntax
The agent can be configured using the following environment variables:

| Variable          | Type                       |    Default     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|-------------------|:---------------------------|:--------------:|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `ROBOT_CODE`      | Any String                 | `"Undefined"`  | A unique code that identifies the robot the agent is listening to. Usually a UUID but any string will work.                                                                                                                                                                                                                                                                                                                                                                                              |
| `SITE_CODE`       | Any String                 | `"Undefined"`  | A unique code that identifies the site of the robot being listened to. Usually a UUID but any string will work.                                                                                                                                                                                                                                                                                                                                                                                          |
| `AGENT_ID`        | Any String                 | `"Undefined"`  | A unique code that identifies the agent itself. Usually a UUID but any string will work.                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `AGENT_MODE`      | `JSON_TEST` or `POST_TEST` |  `JSON_TEST`   | When set to value `JSON_TEST`, will save JSON logs locally on the file system under the `$HOME/.cognicept/agent/logs/< run_id >` folder. Where `< run_id >` is uniquely created every time the agent is launched.                                                                                                       When set to value `POST_TEST`, in addition to saving logs like the `JSON_TEST` mode, will also push the JSON to a REST API endpoint configured by the `AGENT_POST_API` variable. |
| `AGENT_POST_API`  | REST API Endpoint String   | Not applicable | If the `AGENT_MODE` is set to `POST_TEST`, this variable MUST be configured to a valid REST API endpoint. If not specified, the agent will default back to `JSON_TEST` mode. If API endpoint is not available to connect, agent will error out.                                                                                                                                                                                                                                                          |
| `AGENT_TYPE`      | `ROS` or `DB`              |     `ROS`      | When set to `ROS`, the agent catches ANY ROS log that is published to /rosout. When set to `DB`, logs that are only available as part of the *Error Classification System (ECS)* will be considered for reporting, to enable log suppression for particular robots/sites. The ECS should be available for communicating at the REST API endpoint configured by the `ECS_API` variable.                                                                                                                   |
| `ECS_API`         | REST API Endpoint String   | Not applicable | If the `AGENT_TYPE` is set to `DB`, this variable MUST be configured to a valid REST API endpoint. If not specified, the agent will default back to `ROS` mode. If API endpoint is not available to connect, agent will error out.                                                                                                                                                                                                                                                                       |
| `ECS_ROBOT_MODEL` | Valid Robot Model          | Not applicable | If the `AGENT_TYPE` is set to `DB`, this variable MUST be configured to a valid robot model. If not specified, the agent will default back to `ROS` mode. For ROS 2 navigation stack, just use `ROS2_Turtlebot3`.                                                                                                                                                                                                                                                                                             |


**NOTE: To run the agent in the `DB` mode, `ECS API Server` should be running either natively or using Docker. Take a look at the relevant documentation [here][9]. Failure to have the API server will result in the agent not able to find a valid API endpoint and result in an error thrown.**

Based on the type of installation, you can configure these variables by different methods as follows.

### Configure and Run for native installations
In case of a native installation, you can create them using the `export` command at the terminal. For e.g. here is an example set of parameters:

    $ export ROBOT_CODE=R2D2
    $ export SITE_CODE=MFALCON
    $ export AGENT_ID=DROID
    $ export AGENT_MODE=JSON_TEST
    $ export AGENT_TYPE=ROS


**Note: These values are available only in the current terminal and need to be recreated every time before running the listener. One way to get around this is to place these statements in the `bashrc` file**

Now, you can run the listener agent using the provided launch file and `ros2 launch`:

    $ ros2 launch error_resolution_diagnoser_ros2 error_resolution_diagnoser_ros2_launch.py

**This is just a syntax. We will be using this listener agent to connect to a simulation to listen to errors in the next section!**

### Configure and Run for Docker
In case of a Docker installation, you can simply use the [`runtime.env`](runtime.env) file in this repository as an example template and pass it to the docker container with the `--env-file` argument when using the `docker run` command. Simply edit the `runtime.env` like a text file, or comment the unnecessary variables and then rerun the container. Example below:

    $ docker run -it \
    --env-file runtime.env \
    --network=host \
    --name=agent  \
    --volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
    error_resolution_diagnoser_ros2:latest  \
    ros2 launch error_resolution_diagnoser_ros2 error_resolution_diagnoser_ros2_launch.py

**NOTE: This is just a syntax. We will be using this listener agent to connect to a simulation to listen to errors in the next section!**

## Example Application

### Catching Navigation Errors from Navigation 2
In this example, we will run the agent along with the Turtlebot3 simulation to see how ROS 2's [Navigation 2][4] errors are caught.

### Start Simulation
Start a Turtlebot3 Navigation demo as documented [here][2]. Please make sure you have gone through the above documentation if you are facing any errors with this step.

First, open a new terminal and launch the `nav2_simulation_launch` launch file:

    $ export TURTLEBOT3_MODEL=waffle
    $ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
    $ ros2 launch nav2_bringup nav2_simulation_launch.py

**Note: This will open an `RViz2` window. This could be an empty window and needs to be configured. Navigation 2 requires specific configuration as follows.**

In the empty RViz2 window, click on **Startup** button in the bottom left area.

![alt text](/docs/images/startupButton.png "Navigation Startup")

You should now see a map as shown below. To see the sensor information, localize the robot by using the **2D Pose Estimate** button:

![alt text](/docs/images/poseEstimateButton.png "Pose Estimate")

For the purposes of the demonstration, make sure that the robot is intentionally mislocalized. (i.e. the scan doesn't match the map) as shown below:

![alt text](/docs/images/Mislocalized.png "Turtlebot mislocalized")

### Start error_resolution_diagnoser_ros2
We are ready to start listening to robot errors. Based on your installation type, you can start the agent in one of 2 ways:

**Running natively**

Simply launch the agent ROS 2 node using the launch file:

    $ ros2 launch error_resolution_diagnoser_ros2 error_resolution_diagnoser_ros2_launch.py

**Running using Docker**

Run the following `docker run` command:

    $ docker run -it \
    --env-file runtime.env \
    --network=host \
    --name=agent  \
    --volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
    error_resolution_diagnoser_ros2:latest  \
    ros2 launch error_resolution_diagnoser_ros2 error_resolution_diagnoser_ros2_launch.py

 Apart from a few small differences, the agent prompts would look similar for both the types of launches. Sample is shown below:

    [INFO] [launch]: All log files can be found below /home/swaroophs/.ros/log/2020-07-08-15-03-10-766217-swarooph-xps-16391
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [error_resolution_diagnoser_ros2-1]: process started with pid [16523]
    [error_resolution_diagnoser_ros2-1] =======================Environment variables setup======================
    [error_resolution_diagnoser_ros2-1] =========================================================================
    [error_resolution_diagnoser_ros2-1] Environment variable AGENT_TYPE unspecified. Defaulting to ROS mode...
    [error_resolution_diagnoser_ros2-1] Environment variable ROBOT_CODE unspecified. Defaulting to 'Undefined'...
    [error_resolution_diagnoser_ros2-1] Environment variable SITE_CODE unspecified. Defaulting to 'Undefined'...
    [error_resolution_diagnoser_ros2-1] Environment variable AGENT_ID unspecified. Defaulting to 'Undefined'...
    [error_resolution_diagnoser_ros2-1] Environment variable AGENT_MODE unspecified. Defaulting to 'JSON_TEST'...
    [error_resolution_diagnoser_ros2-1] Agent log directory created: /$HOME/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6
    [error_resolution_diagnoser_ros2-1] Updated latest log location in: /$HOME/.cognicept/agent/logs/latest_log_loc.txt
    [error_resolution_diagnoser_ros2-1] TEST mode is ON. JSON Logs will be saved here: /$HOME/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6
    [error_resolution_diagnoser_ros2-1] Subscribed to Listener Agent with direct rosout...
    [error_resolution_diagnoser_ros2-1] Status Logged: Online
    [error_resolution_diagnoser_ros2-1] Status Logged: Online
    [error_resolution_diagnoser_ros2-1] Status Logged: Online
    [error_resolution_diagnoser_ros2-1] Status Logged: Online

Let's unpack what we see on the prompts here. First, we see the `Environment variables setup` section. Here, you can confirm the values of all the environment variables. It will also show if the agent is expecting a particular variable but it was not defined so a default value has been chosen. In our case, none of these variables have been explicitly defined, so the default values are used.

Next, we see the following part:

    [error_resolution_diagnoser_ros2-1] Agent log directory created: /$HOME/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6
    [error_resolution_diagnoser_ros2-1] Updated latest log location in: /$HOME/.cognicept/agent/logs/latest_log_loc.txt
    [error_resolution_diagnoser_ros2-1] TEST mode is ON. JSON Logs will be saved here: /$HOME/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6
    [error_resolution_diagnoser_ros2-1] Subscribed to Listener Agent with direct rosout...

The agent will automatically create a unique `run_id` and then use it to create a folder under `$HOME/.cognicept/agent/logs/run_id` if one does not exist already. This will be where all the logs during a particular session will be stored. This location is also by default stored in a text file `$HOME/.cognicept/agent/logs/latest_log_loc.txt` so that non-ROS based systems can have easy access to the current logs. 

Next, we see the following:

    [error_resolution_diagnoser_ros2-1] Status Logged: Online

The agent not only reports ROS logs such as ERROR, WARN and INFO but also generates *heartbeat* or *status* logs periodically which can be used to ascertain if an agent is "Online" or "Offline". This periodic status is updated every **15 seconds** (not tunable). The physical location of the log can be found at `$HOME/.cognicept/agent/logs/run_id/logDataStatus.json`. For e.g. a sample heartbeat log for Online status is shown below. The `telemetry` field has information from `/amcl_pose` and `/odom` topics if those topics are available. Note also that the timestamp is in `UTC`:

```JSON
{
    "agent_id": "Undefined",
    "compounding": "Null",
    "create_ticket": false,
    "description": "Null",
    "event_id": "Null",
    "level": "Heartbeat",
    "message": "Online",
    "module": "Status",
    "property_id": "Undefined",
    "resolution": "Null",
    "robot_id": "Undefined",
    "source": "Null",
    "telemetry": {
        "nav_pose": {
            "orientation": {
                "w": 0.99987922282419939,
                "x": 0,
                "y": 0,
                "z": -0.015541549616273138
            },
            "position": {
                "x": 1.2097553783353487,
                "y": 1.6474024185674194,
                "z": 0
            }
        },
        "odom_pose": {
            "orientation": {
                "w": 0.99943016586652311,
                "x": -4.9437802107845429e-05,
                "y": 0.0016463737251345726,
                "z": 0.033713952089235048
            },
            "position": {
                "x": -0.34467897230761929,
                "y": -0.44513887853429979,
                "z": 0.0083966415940180308
            }
        }
    },
    "timestamp": "2020-07-08T07:38:57.474978"
}
```
If you keep the agent running, you should be able to see the `[error_resolution_diagnoser_ros2-1] Status Logged: Online` prompt a few more times, once every 15 seconds.

### Generate a navigation error
Now, use `Rviz2` to provide a `Navigation2 goal` for the robot. 

![alt text](/docs/images/NavGoal.png "Navigation2 Goal in Rviz2")

Because the robot is mislocalized, chances are high that it will be unable to reach its goal, generating an error. When that happens, the terminal window running the simulation will show something like the following:

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

The terminal window running the agent will show the following. You can compare this with the prompts above and confirm that the agent is able to receive every message (and then some, since not ALL `rosout` logs are visible on the screen). After receiving, the agent decides to create a JSON log based on suppression logic as to whether that particular log has already been seen before. If it has, it suppresses it. For e.g. log is created only for the first `Spin running...`. The subsequent ones are suppressed. Once the agent receives a `Navigation succeeded` message or message with with `ERROR` level, it resets the suppression logic and makes all logs available for reporting again. This can also be seen with the displayed `event_id` for each log reported. A message is eligible for suppression only within a particular event. And the `event_id` gets reset when the agent receives a `Navigation succeeded` message or message with with `ERROR` level. For e.g. in the scenario below, we start with `event_id` `16764a84-2441-486f-b568-524ae50856e9`. This id is maintained until the FIRST `ERROR` level message is reported. Any new messages after this will have a new `event_id`:

    [error_resolution_diagnoser_ros2-1] Message received: Begin navigating from current location to (-1.40, -0.69)
    [error_resolution_diagnoser_ros2-1] 20 level event logged with id: 16764a84-2441-486f-b568-524ae50856e9
    [error_resolution_diagnoser_ros2-1] /home/swaroophs/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6/logData1.json
    [error_resolution_diagnoser_ros2-1] Message received: Planning algorithm failed to generate a valid path to (-1.40, -0.69)
    [error_resolution_diagnoser_ros2-1] 30 level event logged with id: 16764a84-2441-486f-b568-524ae50856e9
    [error_resolution_diagnoser_ros2-1] /home/swaroophs/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6/logData2.json
    [error_resolution_diagnoser_ros2-1] Message received: Attempting Spin
    [error_resolution_diagnoser_ros2-1] 20 level event logged with id: 16764a84-2441-486f-b568-524ae50856e9
    [error_resolution_diagnoser_ros2-1] /home/swaroophs/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6/logData3.json
    [error_resolution_diagnoser_ros2-1] Message received: Turning -1.57 for spin recovery.
    [error_resolution_diagnoser_ros2-1] 20 level event logged with id: 16764a84-2441-486f-b568-524ae50856e9
    [error_resolution_diagnoser_ros2-1] /home/swaroophs/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6/logData4.json
    [error_resolution_diagnoser_ros2-1] Message received: Spin running...
    [error_resolution_diagnoser_ros2-1] 20 level event logged with id: 16764a84-2441-486f-b568-524ae50856e9
    [error_resolution_diagnoser_ros2-1] /home/swaroophs/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6/logData5.json
    [error_resolution_diagnoser_ros2-1] Status Logged: Online
    [error_resolution_diagnoser_ros2-1] Message received: Spin running...
    [error_resolution_diagnoser_ros2-1] Message received: Spin completed successfully
    [error_resolution_diagnoser_ros2-1] 20 level event logged with id: 16764a84-2441-486f-b568-524ae50856e9
    [error_resolution_diagnoser_ros2-1] /home/swaroophs/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6/logData6.json
    [error_resolution_diagnoser_ros2-1] Message received: Received a goal, begin following path
    [error_resolution_diagnoser_ros2-1] 20 level event logged with id: 16764a84-2441-486f-b568-524ae50856e9
    [error_resolution_diagnoser_ros2-1] /home/swaroophs/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6/logData7.json
    [error_resolution_diagnoser_ros2-1] Message received: Planning algorithm failed to generate a valid path to (-1.40, -0.69)
    [error_resolution_diagnoser_ros2-1] Message received: Attempting Spin
    [error_resolution_diagnoser_ros2-1] Message received: Turning -1.57 for spin recovery.
    [error_resolution_diagnoser_ros2-1] Message received: Goal was canceled. Stopping the robot.
    [error_resolution_diagnoser_ros2-1] 20 level event logged with id: 16764a84-2441-486f-b568-524ae50856e9
    [error_resolution_diagnoser_ros2-1] /home/swaroophs/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6/logData8.json
    [error_resolution_diagnoser_ros2-1] Message received: Spin running...
    [error_resolution_diagnoser_ros2-1] Message received: Spin completed successfully
    [error_resolution_diagnoser_ros2-1] Message received: Planning algorithm failed to generate a valid path to (-1.40, -0.69)
    [error_resolution_diagnoser_ros2-1] Message received: Navigation failed
    [error_resolution_diagnoser_ros2-1] 40 level event logged with id: 16764a84-2441-486f-b568-524ae50856e9
    [error_resolution_diagnoser_ros2-1] /home/swaroophs/.cognicept/agent/logs/66512152-c2af-4384-ba69-47f425ed54a6/logData9.json

**NOTE: `Status Logged: Online` is still being logged concurrently to relay the heartbeat/status.**

Let's look at an example JSON event log, `logData9.json`. Note the `agent_id`, `robot_id` and `property_id` are `Undefined` since they were not set explicitly. If they are set, they will reflect here appropriately. `message` has information about the `rosout` actual message and a unique UUID `event_id`. This also has a flag `create_ticket` that can be used by downstream systems to trigger particular actions such as creating tickets or notifications:

```JSON
{
    "agent_id": "Undefined",
    "compounding": "Null",
    "create_ticket": true,
    "description": "Null",
    "event_id": "16764a84-2441-486f-b568-524ae50856e9",
    "level": "40",
    "message": "Navigation failed",
    "module": "Null",
    "property_id": "Undefined",
    "resolution": "Null",
    "robot_id": "Undefined",
    "source": "bt_navigator",
    "telemetry": {
        "nav_pose": {
            "orientation": {
                "w": 0.40320901162150585,
                "x": 0,
                "y": 0,
                "z": 0.9151079132797445
            },
            "position": {
                "x": 0.9755456306679019,
                "y": 0.76523033522013861,
                "z": 0
            }
        },
        "odom_pose": {
            "orientation": {
                "w": -0.34386135880384766,
                "x": 0.001531624807858039,
                "y": -0.00057754974083514465,
                "z": -0.93901900219501222
            },
            "position": {
                "x": -0.26627073781658595,
                "y": -0.63764414200948083,
                "z": 0.0084155039341280088
            }
        }
    },
    "timestamp": "2020-07-08T07:43:01.618450"
}
```

The `description`, `resolution` and `compounding` fields will be populated only when `AGENT_TYPE` is set to `DB` to provide some context to the errors. 

These JSON logs can be consumed by REST APIs/data streams to connect to incident management/monitoring systems to keep track of robot errors. To showcase this feature, let's use a sample REST API endpoint. Set the `AGENT_POST_API` variable to `https://postman-echo.com`. This is a test API where we can POST our JSON and receive a response for the POST request. Now when you re-run the agent, you will be able to see something similar to the following:

    [error_resolution_diagnoser_ros2-1] Status Logged: Online
    [error_resolution_diagnoser_ros2-1] Posting
    [error_resolution_diagnoser_ros2-1] Pushing downstream...
    [error_resolution_diagnoser_ros2-1] Response: {"args":{},"data":{"agent_id":"Undefined","compounding":"Null","create_ticket":false,"description":"Null","event_id":"Null","level":"Heartbeat","message":"Online","module":"Status","property_id":"Undefined","resolution":"Null","robot_id":"Undefined","source":"Null","telemetry":null,"timestamp":"2020-07-08T08:29:19.986835"},"files":{},"form":{},"headers":{"x-forwarded-proto":"https","x-forwarded-port":"443","host":"postman-echo.com","x-amzn-trace-id":"Root=1-5f0583e0-3518430c3f14972caa50ea40","content-length":"387","content-type":"application/json","user-agent":"cpprestsdk/2.10.2"},"json":{"agent_id":"Undefined","compounding":"Null","create_ticket":false,"description":"Null","event_id":"Null","level":"Heartbeat","message":"Online","module":"Status","property_id":"Undefined","resolution":"Null","robot_id":"Undefined","source":"Null","telemetry":null,"timestamp":"2020-07-08T08:29:19.986835"},"url":"https://postman-echo.com/post"}

From the echo, you are able to see that the response has the same contents as the JSON logs generated. Configure this endpoint appropriately to directly connect the agent to other systems such as incident management. Now, operators can monitor this incident management system to intervene robot operations to correct the errors to reduce downtime on the actual field.

## Related Pages
For more related information, refer to:

* [ECS API Installation][8]
* [ECS API Syntax][9]
* [Getting Started with Navigation 2][1]
* [Turtlebot3 installation][2]
* [ROS 2 log message structure][3]
* [Microsoft C++ REST SDK][5]
* [Docker Installation][6]
* [Intro Document][7]

[1]: https://navigation.ros.org/getting_started/index.html
[2]: https://navigation.ros.org/getting_started/index.html#installation
[3]: https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/Log.msg
[5]: https://github.com/microsoft/cpprestsdk
[6]: https://docs.docker.com/engine/install/ubuntu/
[7]: docs/INTRO.md
[8]: https://github.com/cognicept-admin/error_classification_server#installation
[9]: https://github.com/cognicept-admin/error_classification_server#syntax

## Acknowledgements
We would like to acknowledge the Singapore government for their vision and support to start this ambitious research and development project, *"Accelerating Open Source Technologies for Cross Domain Adoption through the Robot Operating System"*. The project is supported by Singapore National Robotics Programme (NRP).

Any opinions, findings and conclusions or recommendations expressed in this material are those of the author(s) and do not reflect the views of the NR2PO.