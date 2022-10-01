# turtlesim_polygonal_brain

This package allows the turtlesim_node to move around a polyhedron with a given number of sides. **Python 3.8+**

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

source /opt/ros/noetic/setup.bash


echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc

**Installation:**

Clone this package in ~catkin_ws/src/

**Run:**
**One way to start it:**

roslaunch turtlebro_polygonal_brain launch.launch 

**Second way to start it:**

**In first terminal:**

roscore

**In second terminal:**

rosrun turtlesim turtlesim_node

**In third terminal**

rosrun turtlesim_polygonal_brain brain.py 

