# ENPM662 - Robot Modeling
A collection of all projects in ENPM662 'Robot Modeling'

  <h2>Projects list</h2>
    1. ROS package for string subscriber-publisher: <a href="https://github.com/DrKraig/ENPM662/tree/master/src/simple_pubsub"> simple_pubsub</a></br>

xhost +
xhost local:root

docker run -d -it --name ShadowContainer -e interface=eht1 -e DISPLAY=$DISPLAY -e LOCAL_USER_ID=1000 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ~/Documents/Projects/ShadowProject/src/shadowhand:/home/user/workspace/src/shadowhand:rw -v ~/Documents/Projects/ShadowProject/src/shadowlibs:/home/user/workspace/src/shadowlibs:rw -v ~/Documents/Projects/ENPM662/src/project2:/home/user/workspace/src/project2:rw --security-opt seccomp=unconfined --network=host --pid=host --gpus all --privileged --device=/dev:/dev --runtime=nvidia shadow_image:latest

sudo docker exec -u user -it ShadowContainer /bin/bash

roscd sr_config
#sudo chown -R user ~/projects/shadow_robot/base/src/sr_config
#sudo chmod -R user ~/projects/ 

git checkout shadowrobot_170911
rm -rf $(rospack find sr_cyberglove_config)

cd ~/workspace
catkin_make
source devel/setup.bash


roscd shadowhand
cd custom
# Copy only the `shadowhand.world` file and the `sr_right_ur10arm_hand.launch` files.
#sudo chown -R user ~/projects/shadow_robot/base/src/common_resources
#sudo chown -R user ~/projects/shadow_robot/base/src/sr_interface -->

./custom_files_setup.py # press 0 and 2


roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true