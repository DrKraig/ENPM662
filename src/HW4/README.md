# Homework 4
Code for moving a KUKA EE in circle using Inverse Velocity Kinematics

## Dependencies to run the codes
1. Python 3.6 should be installed on your system.
2. numpy - Install it using `python3 -m pip install numpy`
3. matplotlib - Install it using `python3 -m pip install matplotlib`

## Instructions to run the code
  
1. Clone the repository by clicking [here!](https://github.com/DrKraig/ENPM662/tree/devel)
2. Open command prompt or terminal.
3. Navigate to this directory using `cd CMSC756/src/HW4/scripts`
4. To Run problem 1. If OS is Ubuntu, type `python3 problem1.py`
5. To Run problem 1. If OS is Windows, type `python problem1.py`
6. Enjoy!
    
### Instructions for Problem 1
Sample run command:
`python3 problem1.py`

The output will be list of 7 Transformation Matrices from each link frame to ground frame. List of all Inverse jacobians, Joint velocities, and tool frame origin. And a plot of the tool trajectory.