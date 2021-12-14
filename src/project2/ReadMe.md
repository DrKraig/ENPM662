# Dexterous Manipulator

This project consists of the a UR10 manipulator which opens a door using forwad and inverse kinematics. 

# Slides link
The slides can be found [here](https://docs.google.com/presentation/d/1GIJ2iy-gm0YSUg-un2HQpWWHjsj9eKvfluqKY4d7sTo/edit?usp=sharing).

# Dependencies to build the project
 * ROS Melodic on Ubuntu 18.04
 * [Docker](https://docs.docker.com/get-docker/)
 * NVidia Graphic Drivers installed on your local system
 * [Docker Nvidia](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
 * CMake
 * Sympy
# How to build the project
```
Open Terminal

xhost +

xhost local:root

Download the docker image file from https://github.com/DrKraig/ENPM662/blob/devel/docker/project2.Dockerfile

docker build project2.Dockerfile

docker run -d -it --name ShadowContainer -e interface=eht1 -e DISPLAY=$DISPLAY -e LOCAL_USER_ID=1000 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ~/Documents/Projects/ShadowProject/shadowhand:/home/user/workspace/src/shadowhand:rw -v ~/Documents/Projects/ShadowProject/shadowlibs:/home/user/workspace/src/shadowlibs:rw -v ~/Documents/Projects/ShadowProject/project2:/home/user/workspace/src/project2:rw --security-opt seccomp=unconfined --network=host --pid=host --gpus all --privileged --device=/dev:/dev --runtime=nvidia shadow_image:latest

sudo docker exec -u user -it ShadowContainer /bin/bash

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
```

# How to run the project
```
#To run the simulation
#Open 3 terminals
#Go to your workspace and source in all the 3 terminals 
roslaunch shadow_arm door_opener
rosrun shadow_arm starter.py
rosrun shadow_arm grasper.py

#To validate forward_kinematics 
python3 forward_kinematics_validation.py

#To validate inverse_kinematics 
python3 inverse_kinematics_validation.py

```

# Results
<h2> Validating Inverse Kinematics</h2>
<p align="center">
  <img src="./shadow_arm/results/IK_Door_Opening_curve2.png" alt="Logo"/>
</p>

<h2> Grasping the door</h2>
<p align="center">
  <img src="./shadow_arm/results/grasp.png" alt="Logo"/>
</p>
<h2>Opening the door</h2>
<p align="center">
  <img src="./shadow_arm/results/door_opening.png" alt="Logo"/>
</p>


# References & Some important topics for debugging

 * [Shadow Hand Documentation](https://dexterous-hand.readthedocs.io/en/latest/user_guide/3_software_description.html#writing-controllers)

 * [Shadow Hand Documentation 2](https://buildmedia.readthedocs.org/media/pdf/dexterous-hand/fsrc-2786_documentation/dexterous-hand.pdf) #Page 44

 * Topic is: /contacts/rh_th/proximal. Message is: http://docs.ros.org/en/api/gazebo_msgs/html/msg/ContactState.html

* Joint Names for arm and hand [ra_elbow_joint, ra_shoulder_lift_joint, ra_shoulder_pan_joint, ra_wrist_1_joint, ra_wrist_2_joint, ra_wrist_3_joint, rh_FFJ1, rh_FFJ2, rh_FFJ3, rh_FFJ4, rh_LFJ1, rh_LFJ2, rh_LFJ3, rh_LFJ4, rh_LFJ5, rh_MFJ1, rh_MFJ2, rh_MFJ3, rh_MFJ4, rh_RFJ1,rh_RFJ2, rh_RFJ3, rh_RFJ4, rh_THJ1, rh_THJ2, rh_THJ3, rh_THJ4, rh_THJ5, rh_WRJ1,
  rh_WRJ2]

 * Publish joint positions to this topic. rostopic echo /rh_trajectory_controller/command 

# Developers
 * Pavan Mantripragada
 * Sameer Pusegaonkar
