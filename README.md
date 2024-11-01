# Turtlebot Simulation

This repository contains ROS packages for controlling a robot model TurtleBot3 Burger. The robot can be commanded to reach any specified coordinate using a PID controller that controls its speed.

## Features:

- **PID Controller for Stable Movement**: To ensure smooth and stable movement of the two-wheeled Turtlebot3 and a two-legged robot Darwin, a Proportional-Integral-Derivative (PID) controller has been implemented. This controller helps regulate the robot's velocity and direction, enhancing its ability to follow the planned path accurately.

- **Artificial Potential Field Algorithm**: Implementation of artificial potential field algorithm, a popular technique in robotic path planning. It creates a virtual force field around obstacles, guiding the robot away from them while attracting it towards the target destination.

- **Obstacle Avoidance in Dynamic Environments**: The system incorporates coordinate transformation techniques to facilitate obstacle avoidance in dynamic simulation environments. By dynamically updating the potential field based on the robot's sensor readings, it adapts to changes in the surroundings in real-time.

## APF Simulation Packages

- **APF_simulation**: Contains launch files and configuration for simulating the Darwin robot in Gazebo and the Python script for controlling the Darwin robot to reach specified coordinates.

## Running Node under APF_simulation Package

1. **Turtlebot3 setup**: Use Robotis Emanual to setup the Turtlebot: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

2. **Create a Workspace**: Create a new ROS workspace.
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```

3. **Clone the Repository**: Clone this repository into the `src` directory of your ROS workspace.
    ```bash
    git clone https://github.com/t-kadre/TurtleBot3-APF-Simulation.git
    ```

4. **Build the Workspace**: Build the ROS workspace using `catkin_make`.
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

5. **Source the Workspace**: Source the setup script of your workspace.
    ```bash
    source devel/setup.bash

6. **Set Executable Permissions**: Navigate to the APF_Simulation_Packages folder and then to the CMakeLists.txt file to set executable permissions for the required cpp scripts. Add the following lines:
   
    ```bash
    add_executable(mynode_11 src/scripts/mynode_03.cpp)
    target_link_libraries(mynode_03 ${catkin_LIBRARIES})

    add_executable(mynode_11 src/scripts/mynode_08.cpp)
    target_link_libraries(mynode_08 ${catkin_LIBRARIES})

    add_executable(mynode_11 src/scripts/mynode_11.cpp)
    target_link_libraries(mynode_11 ${catkin_LIBRARIES})
    ```
 7. **Launch Gazebo Simulation**: Launch the Gazebo simulation environment for the Turtlebot3 robot from a separate terminal and sourcing the bash file .

 8. **Obstacle Setup**: set up random obstacles in gazebo simulation using the available object

 9. **Run min.distance obstacle array Node**: In another terminal, run the mynode_03.cpp script to finds min. distance obstacle array from live turtlebot3 lidar data.
  
    ```bash
    cd ~/catkin_ws/src
    source devel/setup.bash
    rosrun APF_simulation mynode_03
    ```
    
 10. **Run specified coordinate and angle turn Node**: In another terminal, run the mynode_08.cpp script to to move the turtlebot to a specified coordinate at specific        angle.

     ```bash
     cd ~/catkin_ws/src
     source devel/setup.bash
     rosrun APF_simulation mynode_08
     ```

 11. **Run apf_simulation Node**: In another terminal, run the mynode_11.cpp script to to move the turtlebot to a goal position (10,10) avoiding obstacles.

     ```bash
     cd ~/catkin_ws/src
     source devel/setup.bash
     rosrun APF_simulation mynode_11
     ```
