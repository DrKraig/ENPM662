# ENPM662 - Robot Modeling
A collection of all projects in ENPM662 'Robot Modeling'

  <h2>Projects list</h2>
    1. ROS package for string subscriber-publisher: <a href="https://github.com/DrKraig/ENPM662/tree/master/src/simple_pubsub"> simple_pubsub</a></br>

`docker run -d --name ShadowContainer -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ~/Documents/Projects/ENPM662/src/project2:/home/user/workspace/src/project2:rw  --net=host --privileged --device=/dev:/dev --runtime=nvidia shadow_image:latest`

xhost +
xhost local:root

`docker run -d --name ShadowContainer -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ~/Documents/Projects/ShadowProject/src/shadowhand:/home/user/workspace/src/shadowhand:rw -v ~/Documents/Projects/ShadowProject/src/shadowlibs:/home/user/workspace/src/shadowlibs:rw -v ~/Documents/Projects/ENPM662/src/project2:/home/user/workspace/src/project2:rw --net=host --privileged --device=/dev:/dev --runtime=nvidia shadow_image:latest`

roscd sr_config
git checkout shadowrobot_170911
rm -rf $(rospack find sr_cyberglove_config)


cd ~/workspace
catkin_make
source devel/setup.bash


roscd shadowhand
cd custom
# Copy only the `shadowhand.world` file and the `sr_right_ur10arm_hand.launch` files.
./custom_files_setup.py # press 0 and 2


run `roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true`
