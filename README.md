# Mobile Robotics Course Assessment Task - 2023
# Heiko Schoon, Matr.-Nr. 7013850

Autonomous robot that builds a map of the maze (world), identifes the yellow block and reports the position of this block back by posting a specific marker message.

For the creation of the world it is necessary to modify the GAZEBO_MODEL_PATH in .bashrc:
**export GAZEBO_MODEL_PATH=~/ros2_ws/src/mobile_robotics_hschoon/meshes**

The package "mobile_robotics_hschoon" requires the explore-lite package included in the m-explore-ros2 folder.
It is important to use the explore-lite files from this .zip because of modified parameters in the param.yaml.
The package can be started via the script "ros_exam_auto.sh" in the autonomous explorer mode (start from ~/ros2_ws directory).


Included in the mobile_robotics_hschoon package are:
* Package.xml and CMakeLists.txt files
* launch directory, including necessary launch files
* urdf directory, including the urdf, sdf and xacro files for your robot
* meshes directory, including any mesh files needed for the robot model
* config directory, including required YAML files
* scripts directory, including any written Python source code files
* worlds directory, including the prepared World file for the task
* rviz directory, including the prepared RVIZ configuration file
