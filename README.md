# Steps to launch the simulation 
Create your workspace and git clone in your src folder using the following command:


git clone https://github.com/ShishirV712/URC_NAV.git


Navigate back to the workspace directory and type the following commands:


colcon build 


source install/setup.bash


Now type:


ros2 launch rover_gazebo mars.launch.py
