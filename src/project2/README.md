# ENPM662 - Project 2


## Docker Setup

### Run this command on host machine to give access for your X server
`xhost +`
`xhost local:root`

### Run this command to create your container and start for the first time
`docker run -d -it --name ShadowContainer -e interface=eht1 -e DISPLAY=$DISPLAY -e LOCAL_USER_ID=1000 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ~/Documents/Projects/ShadowProject/src/shadowhand:/home/user/workspace/src/shadowhand:rw -v ~/Documents/Projects/ShadowProject/src/shadowlibs:/home/user/workspace/src/shadowlibs:rw -v ~/Documents/Projects/ENPM662/src/project2:/home/user/workspace/src/project2:rw --security-opt seccomp=unconfined --network=host --pid=host --gpus all --privileged --device=/dev:/dev --runtime=nvidia shadow_image:latest`

### Instructions for the first time setup

Enter following commands in the terminator window opened

`roscd sr_config`
#sudo chown -R user ~/projects/shadow_robot/base/src/sr_config
#sudo chmod -R user ~/projects/ 

`git checkout shadowrobot_170911`
`rm -rf $(rospack find sr_cyberglove_config)`

`cd ~/workspace`
`catkin_make`
`source devel/setup.bash`


`roscd shadowhand`
`cd custom`
# Copy only the `shadowhand.world` file and the `sr_right_ur10arm_hand.launch` files.
#sudo chown -R user ~/projects/shadow_robot/base/src/common_resources
#sudo chown -R user ~/projects/shadow_robot/base/src/sr_interface -->

`./custom_files_setup.py` # press 0 and 2

### Instruction to launch the shadowrobot in simulation

After the setup, run this command anytime after starting the container 

`roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true`

### To get into the container when it is already started 
sudo docker exec -u user -it ShadowContainer /bin/bash


### Grasping pipeline

We have 4 different door knobs.

1. Flat push
2. Pull handle
3. L knob
4. Ball knob

Design a grasp controller using joint state controllers

We will manually place the hand near the object. With some hardcoded pre-grasp pose
and then we will pick one of the pre-grasp pose for each object from which the joint state controller succeds. (Need to fill in why we chose that pose using kinematic analysis).

First spawn the door -> manually set the pre-grasp -> allow the controller to do its job -> if succeded use the pre-grasp else repeat until success.