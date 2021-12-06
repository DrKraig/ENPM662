#xhost +
#xhost local:root

docker run -d -it --name ShadowContainer -e interface=eht1 -e DISPLAY=$DISPLAY -e LOCAL_USER_ID=1000 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ~/Documents/Projects/ShadowProject/shadowhand:/home/user/workspace/src/shadowhand:rw -v ~/Documents/Projects/ShadowProject/shadowlibs:/home/user/workspace/src/shadowlibs:rw -v ~/Documents/Projects/ShadowProject/project2:/home/user/workspace/src/project2:rw --security-opt seccomp=unconfined --network=host --pid=host --gpus all --privileged --device=/dev:/dev --runtime=nvidia shadow_image:latest

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

###############################################################
Next Task
#Check the grapsping manually of objects
#Torque sensors for feedback ->topic using echo

https://dexterous-hand.readthedocs.io/en/latest/user_guide/3_software_description.html#writing-controllers 

https://buildmedia.readthedocs.org/media/pdf/dexterous-hand/fsrc-2786_documentation/dexterous-hand.pdf
#Page 44



#Backup is Contact State. 
Topic is: /contacts/rh_th/proximal
Message is: http://docs.ros.org/en/api/gazebo_msgs/html/msg/ContactState.html

###############################################################
#Write custom launch file to bring in own objects
#COntrol atleast one single joint by publishing to the joint state controller   

###############################################################
# To run the door opener
roslaunch shadow_arm door_opener.launch

name: [ra_elbow_joint, ra_shoulder_lift_joint, ra_shoulder_pan_joint, ra_wrist_1_joint,
  ra_wrist_2_joint, ra_wrist_3_joint, rh_FFJ1, rh_FFJ2, rh_FFJ3, rh_FFJ4, rh_LFJ1,
  rh_LFJ2, rh_LFJ3, rh_LFJ4, rh_LFJ5, rh_MFJ1, rh_MFJ2, rh_MFJ3, rh_MFJ4, rh_RFJ1,
  rh_RFJ2, rh_RFJ3, rh_RFJ4, rh_THJ1, rh_THJ2, rh_THJ3, rh_THJ4, rh_THJ5, rh_WRJ1,
  rh_WRJ2]
position: [0.9584337832687053, -0.9983795245982279, 0.016947453303541415, -0.009201515067289279, 1.5959315355593153, -1.6216245456501603, 1.2131067141041285e-05, 6.137642353998274e-05, -2.6818967525876758e-05, 5.442464555027726e-05, 9.735529790066977e-05, 6.003782756369702e-05, -9.373317795091651e-05, -9.05622935407635e-05, -5.950816632083189e-05, 4.223102800970935e-05, 6.205215736798664e-05, -4.8952747785158124e-05, 3.724180018060963e-05, 4.189128643083961e-06, 0.0001097130418230563, 8.013298936404567e-05, -1.6813940607818267e-05, -3.418510038155631e-05, 0.00032800853396874885, 0.0001355905323148221, -0.014368843279465082, 6.240253875411383e-05, -4.790583458191833e-05, -0.03902219842275301]

Pub to this
#rostopic echo /rh_trajectory_controller/command 
joint_names: [rh_FFJ1, rh_FFJ2, rh_FFJ3, rh_FFJ4, rh_LFJ1, rh_LFJ2, rh_LFJ3, rh_LFJ4, rh_LFJ5,
  rh_MFJ1, rh_MFJ2, rh_MFJ3, rh_MFJ4, rh_RFJ1, rh_RFJ2, rh_RFJ3, rh_RFJ4, rh_THJ1,
  rh_THJ2, rh_THJ3, rh_THJ4, rh_THJ5, "rh_WRJ1", "rh_WRJ2"]
points: 
  - 
    positions: [0.00011875388677040633, 0.03490658503988659, 0.767944870877505, 0.3490658503988659, 0.0003180250455327993, 0.017453292519943295, 0.06981317007977318, -0.3490658503988659, 0.15707963267948966, 0.0003293390335734614, 0.03490658503988659, 0.17453292519943295, 0.3490658503988659, -0.00048823215790960717, 0.03490658503988659, 0.17453292519943295, -0.3490658503988659, 0.10471975511965978, 0.6981317007977318, 0.03490658503988659, 1.2217304763960306, 0.5585053606381855, 0.4886921905584123, -0.5235987755982988]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#Gripper positions. DONT DELETE
positions: [0.0013002309803011869, -0.0005731183652812888, 1.5707963267948966, -0.0040847566611059705, -0.002831573192481507, 0.0039904283167455645, 1.5707963267948966, -0.005421899417669351, -0.0003065840215104032, -8.145947540683096e-05, 0.0006961220194154905, 1.5707963267948966, -0.003671902003530292, 0.00044755422390529986, 0.0001075421982310587, 1.5707963267948966, -0.003954332143575989, 0.3665191429188092, -0.6981317007977318, -0.20943951023931956, 1.2217304763960306, 0.0, 0.00011653269466460614, -0.017453292519943295]

