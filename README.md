# Cheongju University LiDAR Lecture Notes<br>
  Author : Hwasu Lee<br><br>
  Affiliation : UNICON LAB (Incheon National University, South Korea)<br><br>
  Position : M.A. student<br><br>
  E-mail : leehwasu96@naver.com (or leehwasu9696@inu.ac.kr)<br><br>

#  Course Duration
  Date : 24.07.02 ~ 24.07.04<br><br>

# LiDAR Lecture Note Description
  "LiDAR Lecture Notes (1).pdf is the LiDAR lecture notes."<br><br>
  "LiDAR Lecture Notes (2).pdf is the LiDAR lecture notes."<br><br>
  "LiDAR Lecture Homework.pdf is the LiDAR lecture homeworks."<br><br>

# SLAM(Simultaneous Localization And Mapping) practice

<br>

**Note: This practice was conducted in an Ubuntu 20.04 LTS and ROS(Robot Operating System) 1 Noetic environment.** <br><br><br>

**To set up the project, follow these steps:** <br><br><br>

**1. Create catkin workspace and src folder** <br>
  ```shell
  mkdir -p ~/catkin_ws/src
  ```
<br><br>

**2. Change directory to the src folder within the catkin workspace** <br>
  ```shell
  cd ~/catkin_ws/src
  ```
<br><br>

**3. Initialize and set up the build environment for the catkin workspace** <br>
  ```shell
  catkin_init_workspace
  ```
<br><br>

**4. Build the catkin workspace** <br>
  ```shell
  cd ~/catkin_ws && catkin_make
  ```
<br><br>

**5. Add the ROS package path to the environment variables** <br>
  ```shell
  source devel/setup.bash
  ```
<br><br>

**6. Change directory to the src folder within the catkin workspace** <br>
  ```shell
  cd ~/catkin_ws/src
  ```
<br><br>

**7. Install the Turtlebot3-related packages** <br>
  ```shell
  git clone –b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
  ```
  ```shell
  git clone –b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  ```
<br><br>

**8. Install the Turtlebot3-related packages and then build the catkin workspace** <br>
  ```shell
  cd ~/catkin_ws && catkin_make
  ```
<br><br>

**9. Define the Turtlebot3 model before running the Gazebo simulation** <br>
  ```shell
  export TURTLEBOT3_MODEL=burger
  ```
<br><br>

**10. Run the Turtlebot3 environment based on the Gazebo simulation** <br>
  ```shell
  roslaunch turtlebot3_gazebo turtlebot3_world.launch
  ```
  **or**
  ```shell
  roslaunch turtlebot3_gazebo turtlebot3_house.launch
  ```
<br><br>

**11. Run the code to control the Turtlebot3 model in a terminal different from the one where the Turtlebot3 world is running** <br>
  ```shell
  export TURTLEBOT3_MODEL=burger
  ```
  ```shell
  roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
  ```
<br><br>

**12. Run the code to perform SLAM(Gmapping) in a terminal different from the ones running the Turtlebot3 world and Turtlebot3 teleop_key** <br>
  ```shell
  export TURTLEBOT3_MODEL=burger
  ```
  ```shell
  roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  ```
<br><br>

**13. Save the Occupancy Grid Map created based on SLAM(Gmapping) using the map_server** <br>
  ```shell
  rosrun map_server map_saver -f ~/map
  ```
<br><br>

**13-1. If the map_server package is not available, install it using the following command** <br>
  ```shell
  sudo apt update
  ```
  ```shell
  sudo apt install ros-noetic-map-server
  ```
<br><br>

# ROS Navigation Stack practice command

<br>

**Note: This practice was conducted in an Ubuntu 20.04 LTS and ROS(Robot Operating System) 1 Noetic environment.** <br><br><br>

**To set up the project, follow these steps:** <br><br>

**1. Define the Turtlebot3 model before running the Gazebo simulation** <br>
  ```shell
  export TURTLEBOT3_MODEL=burger
  ```
<br><br>

**2. Run the Turtlebot3 environment based on the Gazebo simulation in the same terminal** <br>
  ```shell
  roslaunch turtlebot3_gazebo turtlebot3_world.launch
  ```
  **or**
  ```shell
  roslaunch turtlebot3_gazebo turtlebot3_house.launch
  ```
<br><br>

**3. In a different terminal, run the following code to practice the Navigation Stack for the Turtlebot3 model** <br>
  ```shell
  export TURTLEBOT3_MODEL=burger
  ```
  ```shell
  roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/unicon3/map.yaml
  ```
  **In the above command, modify that path to the path where your map.yaml file is located.** <br>
  **/home/unicon3/map.yaml --> /home/{your map.yaml path}**
<br><br><br>

**4. In Rviz (ROS Visualization), set the initial position of the Turtlebot3 Burger model using the "2D Pose Estimate" button, then click "2D Nav Goal" to set the target position for the Turtlebot3 Burger model.** <br><br><br>

# Homework : <br>Control the Turtlebot3 in the ROS Gazebo simulation environment

<br>

**Note: This practice was conducted in an Ubuntu 20.04 LTS and ROS(Robot Operating System) 1 Noetic environment.** <br><br><br>

**To set up the project, follow these steps:** <br><br><br>

**1. Create the turtlebot3_control package and the scripts folder** <br>
  ```shell
  cd ~/catkin_ws/src
  ```
  ```shell
  catkin_create_pkg turtlebot3_control rospy std_msgs sensor_msgs geometry_msgs
  ```
  ```shell
  cd ~/catkin_ws/src/turtlebot3_control
  ```
  ```shell
  mkdir scripts
  ```
  ```shell
  cd scripts
  ```
<br><br>

**2. To control the Turtlebot3 model, fetch the "drive_and_stop.py" and "drive_and_turn_and_stop.py" file from the Homework directory in the Github repository and place it in the scripts folder.**
<br><br><br>

**3. Grant execute permissions to the Python file fetched from Github** <br>
  ```shell
  chmod +x drive_and_stop.py drive_and_turn_and_stop.py 
  ```
<br><br>

**4. Change directory to the "worlds" folder within the "turtlebot3_gazebo" package** <br>
  ```shell
  roscd turtlebot3_gazebo/worlds
  ```
<br><br>

**5. Fetch the "custom_world.world" file from the Homework folder in the Github repository and save it to the above path.**
<br><br><br>

**6. Change directory to the "launch" folder within the "turtlebot3_gazebo" package** <br>
  ```shell
  roscd turtlebot3_gazebo/launch
  ```
<br><br>

**7. Copy an existing launch file to run the "custom_world.world" file** <br>
  ```shell
  cp turtlebot3_empty_world.launch turtlebot3_custom_world.launch
  ```
<br><br>

**8. Modify the copied "turtlebot3_custom_world.launch" file** <br>
  ```shell
  gedit turtlebot3_custom_world.launch
  ```
**Modify from "empty_world.world" to "custom_world.world".** <br>
**Once the modifications are complete, save and close the file using the 'ctrl+s' shortcut.**
<br><br><br>

**9. Build the catkin workspace and set the ROS environment variables** <br>
  ```shell
  cd ~/catkin_ws && catkin_make
  ```
  ```shell
  source devel/setup.bash && source /opt/ros/noetic/setup.bash
  ```
<br><br>

**10. In the first terminal, define the Turtlebot3 model and run the Gazebo simulation environment** <br>
  ```shell
  export TURTLEBOT3_MODEL=burger
  ```
  ```shell
  roslaunch turtlebot3_gazebo turtlebot3_custom_world.launch
  ```
<br><br>

**11. Run the First control practice code based on the Turtlebot3 Burger model and LiDAR sensor data** <br>
  ```shell
  rosrun turtlebot3_control drive_and_stop.py
  ```
  **or**
  ```shell
  cd ~/catkin_ws/src/turtlebot3_control/scripts
  ```
  ```shell
 python drive_and_stop.py
  ```
<br><br>

**Turtlebot3 Control example video 1** <br>

https://github.com/user-attachments/assets/a5226ede-281f-49f7-ad6d-d52746bdae5d

<br><br>

**12. Run the Second control practice code based on the Turtlebot3 Burger model and LiDAR sensor data** <br>
  ```shell
  rosrun turtlebot3_control drive_and_turn_and_stop.py
  ```
  **or**
  ```shell
  cd ~/catkin_ws/src/turtlebot3_control/scripts
  ```
  ```shell
 python drive_and_turn_and_stop.py
  ```
<br><br>

**Turtlebot3 Control example video 2** <br>

https://github.com/user-attachments/assets/a41602c4-c304-470c-b5c7-26afd393852d

<br><br>
