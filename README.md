# KONE.533-Automated-Data-Generation-Setup
Created as a project work for the Tampere University course KONE.533

A tool for controlling a MoveIt integrated robot arm with an attached camera.
Camera should publish to a ROS topic.

Basic usage:
1. Set up your robot for MoveIt
2. Run movegen.py
3. Run moveit_control.py
4. Run bmaskgen.py

Each file above contains some parameters to me modified depending on desired results, cameras and coordinate systems. Some more information is provided within.

Designed to be used with Franka Emika Panda and it's associated default configurations:
https://github.com/frankaemika/franka_ros
https://github.com/ros-planning/panda_moveit_config

With these files, only camera parameters and scaling in bmaskgen.py should absolutely require changes.
