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

# SLAM & Navigation practice command

Please note that this practice was conducted in an Ubuntu 20.04 LTS and ROS 1 Noetic environment.

To set up the project, follow these steps:

1. Create catkin workspace and src folder:
  ```shell
  mkdir -p ~/catkin_ws/src
  ```
<br>

2. Change directory to the src folder within the catkin workspace:
  ```shell
  cd ~/catkin_ws/src
  ```
<br>

3. Initialize and set up the build environment for the catkin workspace:
  ```shell
  catkin_init_workspace
  ```
<br>

4. Build the catkin workspace:
  ```shell
  cd ~/catkin_ws && catkin_make
  ```
<br>

5. Add the ROS package path to the environment variables:
  ```shell
  source devel/setup.bash
  ```
<br>

6. Change directory to the src folder within the catkin workspace:
  ```shell
  cd ~/catkin_ws/src
  ```
<br>

7. Install the Turtlebot3-related packages:
  ```shell
  git clone –b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
  ```
  ```shell
  git clone –b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  ```
<br>

8. Install the Turtlebot3-related packages and then build the catkin workspace:
  ```shell
  cd ~/catkin_ws && catkin_make
  ```
<br>

9. Define the Turtlebot3 model before running the Gazebo simulation:
  ```shell
  export TURTLEBOT3_MODEL=burger
  ```
<br>

10. Run the Turtlebot3 environment based on the Gazebo simulation:
  ```shell
  roslaunch turtlebot3_gazebo turtlebot3_world.launch
  ```
<br>
