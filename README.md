# Mobile Robotics Course Assessment Task - 2023

Autonomous robot that builds a map of the maze (world), identifes the yellow block and reports the position of this block back by posting a specific marker message.

Included in the package are:
* Package.xml and CMakeLists.txt files
* launch directory, including necessary launch files
* urdf directory, including the urdf, sdf and xacro files for your robot
* meshes directory, including any mesh files needed for the robot model
* config directory, including required YAML files
* src directory, including any written C source code files
* scripts directory, including any written Python source code files
* worlds directory, including the prepared World file for the task
* rviz directory, including the prepared RVIZ configuration file

The bash-script (ros_exam.sh) contains the commands necessary to launch the package for manual driving and mapping (31.05.23).
For the creation of the world it is necessary to modify the GAZEBO_MODEL_PATH in .bashrc:
**export GAZEBO_MODEL_PATH=~/ros2_ws/src/mobile_robotics_hschoon/meshes**

- [X] Robot designed and loads into world.
- [X] Basic moving possible and dimensions (physics) adapted.
- [X] Included libgazebo_ros_camera.
- [ ] Autonomous movement
- [ ] Map building
- [ ] Identifying the position of the yellow square and publishing it in the correct format.
