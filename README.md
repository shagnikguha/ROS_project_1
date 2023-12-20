# ROS Robot Motion Planning and Navigation
This project utilizes ROS Noetic, Gazebo, Rviz, and XML to address various challenges in robot motion planning and navigation. Below is an overview of the key components and functionalities of the project.

## Project Structure
### 1. 'motion_plan' Folder
- This folder contains code for operating the robot to center itself between two walls.
- It includes obstacle avoidance logic and manual calculation of odometric data from IMU outputs.
- Launch files for various functionalities, including GMapping, are provided.
- 2D maps and 3D world data are also included in this folder.

### 2.'m2wr_description' Folder
- The urdf file in this folder contains the robot's Unified Robot Description Format (URDF) code.
- Various plugins are integrated to enable the usage of Lidar, Cameras, IMU, and a differential drive.
- Launch files for Gazebo and Rviz are included, along with custom 3D models.

### 3. 'navigation' Folder
- This folder contains codes to configure Simultaneous Localization and Mapping (SLAM).
- YAML files specify the parameters for how the robot should move.

## How to Use
#### 1. Motion Planning
- Navigate to the motion_plan folder.
- Run the desired launch file to execute specific functionalities, such as GMapping.
#### 2. Robot Description:
- Explore the m2wr_description folder to understand the URDF code and custom 3D models.
- Use the provided launch files to visualize the robot in Gazebo and Rviz.
#### 3. Navigation:
- Visit the navigation folder to configure SLAM using YAML files.
