# ENPM662 - Project 1 
Castorbot ROS package

## Dependencies to run the codes

1. Python 2 should be installed on your system.
2. ROS melodic full-desktop installation 
3. Run `$ sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers`

## Instructions to build the package 
1. Download the attachment on Canvas to your local machine
2. Unzip the attachment to find the "castorbot" package along with report, assembly files and video submissions.
3. Or clone the repository by clicking [here!](https://github.com/DrKraig/ENPM662) to find the "castorbot" package in `/src` directory.
4. Place the package in your catkin_ws
5. Open a new terminal and got to your catkin_ws by running `$ cd ~/catkin_ws`
6. Run `$ catkin build` or `$ catkin_make` to build the package.
7. To source ROS, make sure you run `$ source ~/catkin_ws/devel/setup.bash` in all the terminals you open 

## Instructions to run the castorbot teleop
  
1. Open 2 command prompts or terminals and source ROS.
2. Run `$ roslaunch castorbot template_launch.launch` in the first terminal to spawn castorbot in the gazebo env.
3. In the second terminal, run `$ rosrun castorbot teleop_template.py` to start teleop node.
4. Press 'Q' to increase target velocity.
5. Press and hold 'U','I','O' to gradually see the controller acheive the target speed.
6. Use 'U' to move forward-left. 'I' to move forward. 'O' to move forward-right.
7. Press 'K' to stop the bot.
8. Enjoy!

## Instructions to run the castorbot publisher/subscriber

1. Open 3 command prompts or terminals and source ROS.
2. Run `$ roslaunch castorbot template_launch.launch` in the first terminal to spawn castorbot in the gazebo env.
3. In the second terminal, run `$ rosrun castorbot circle_subscriber.py` to start subscriber node.
4. In the third terminal, run `$ rosrun castorbot circle_publisher.py` to start publisher node.
5. See the bot moving in a circle and stop after a while!