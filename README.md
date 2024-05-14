# Path Tracking in ROS using Pure Pursuit & MPC Algorithm

![rviz_overview](https://github.com/KAN201197/Path_Tracking/assets/128454220/a1c0f9b5-7f51-4a2d-8e41-f76909842186)

This repository implements two different algorithms for path tracking in ROS: Model Predictive Control (MPC) and Pure Pursuit. Both algorithms are designed to guide a robot along a predefined path while maintaining stability and accuracy.

## Introduction

Path tracking is an essential capability for autonomous mobile robots, enabling them to follow a desired trajectory accurately. This repository provides a ROS-based implementation of two popular path tracking algorithms:
- **Model Predictive Control (MPC)**
- **Pure Pursuit**

## Features

- **Model Predictive Control (MPC)**: Uses a mathematical model to predict the future states of the robot (based on horizontal horizon) and optimize control inputs accordingly.
- **Pure Pursuit**: A geometric algorithm that computes the necessary steering angle to reach a lookahead point on the path.
- **PID Controller**: Used to control linear velocity of the robot.
- **ROS Integration**: Fully integrated with ROS for real-time operation.
- **Dynamic Reconfigure**: Allows runtime parameter tuning.

## Prerequisites
* System Requirements:
  * Ubuntu 20.04
  * ROS Noetic
  * C++11 and above
  * CMake: 3.0.2 and above
  * ACADO (can be installed through this link: https://github.com/acado/acado)
* This repo depends on the following standard ROS pkgs:
  * `roscpp`
  * `rospy`
  * `rviz`
  * `std_msgs`
  * `nav_msgs`
  * `geometry_msgs`
  * `visualization_msgs`
  * `tf2`
  * `tf2_ros`
  * `tf2_eigen`
  * `tf2_geometry_msgs`
  * `gazebo_ros`
  * `jsk_rviz_plugins`
  * `jackal_gazebo`
  * `jackal_navigation`
  * `velodyne_simulator`
  * `dynamic_reconfigure`

## Building the Package (Installation)

This repo is a ros workspace, containing two rospkgs:
* `me5413_world` the main pkg containing the gazebo world, source code, and the launch files
* `jackal_description` contains the modified jackal robot model descriptions

1. Build ROS workspace and Clone the repository:

    ```bash
    mkdir ~catkin_ws/src -p
    
    cd catkin_ws/src
    
    git clone https://github.com/KAN201197/Path_Tracking.git
    
    cd ..
    
    rosdep update
    
    rosdep install --from-paths src --ignore-src -r -y
    ```

2. Build the package using `catkin_make`:

    ```bash
    cd ~/catkin_ws
    
    catkin_make
    
    source devel/setup.bash
    ```

## Usage

### Running the Nodes

1. Launch the ROS master:

    ```bash
    roscore
    ```

2. In a new terminal, launch the gazebo world:
   
    ```bash
    roslaunch me5413_world world.launch
    ```
    ![image](https://github.com/KAN201197/Path_Tracking/assets/128454220/bfe37ae9-bece-4b41-ba05-9e9761253460)

4. In a new terminal, launch the MPC path tracker:

    ```bash
    roslaunch me5413_world path_tracking_MPC.launch
    ```

5. If you want to run other path tracking algorithm, launch the Pure Pursuit path tracker:

    ```bash
    roslaunch me5413_world path_tracking.launch
    ```
   ![2024-05-1420-16-16-ezgif com-video-to-gif-converter](https://github.com/KAN201197/Path_Tracking/assets/128454220/76e790c6-0883-49c9-8a6a-5b261d840b5e)

## Configuration

![image](https://github.com/KAN201197/Path_Tracking/assets/128454220/4e8e2954-4bd2-46b9-92f5-ccc425527b8f)

The dynamic reconfigure server allows you to adjust parameters at runtime. You can use rqt_reconfigure to tune the following parameters:

- speed_target: Desired speed of the robot.
- robot_length: Length of the robot for kinematic modeling.
- PID_Kp, PID_Ki, PID_Kd: PID controller gains.
- lookahead_distance: Distance to the lookahead point.
