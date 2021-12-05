#include <ros/ros.h>
#include <math.h>
#include <numeric> 
#include <string>
#include <shadowlibs/shadow_planning_options.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_interface/planning_interface.h>

bool planToPoseTarget(
    shadow_planning::PlanningOptions &options,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    geometry_msgs::Pose &target_pose, std::string &reference_frame,
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    std::string &end_effector_name) {
  move_group_interface.clearPoseTargets();
  move_group_interface.setPlanningTime(5.0); // options.set_planning_time
  move_group_interface.allowReplanning(options.allow_replanning);
  move_group_interface.setNumPlanningAttempts(options.num_attempts);
  move_group_interface.setMaxAccelerationScalingFactor(options.acceleration_scaling_factor);
  move_group_interface.setMaxVelocityScalingFactor(options.velocity_scaling_factor);
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  move_group_interface.setPoseTarget(target_pose);
  if (reference_frame != "") {
    move_group_interface.setPoseReferenceFrame(reference_frame);
  }
  if (end_effector_name != "") {
    move_group_interface.setEndEffector(end_effector_name + "_ee");
  }
  move_group_interface.setPlannerId("TRRTkConfigDefault");
  ROS_INFO("Planning for: %s", move_group_interface.getEndEffector().c_str());

  // Do planning for entire group
  bool plan_success = false;
  plan_success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // Return success or failure
  if (plan_success) {
    ROS_INFO("Plan Succeeded");
  } else {
    ROS_INFO("Plan Failed");
  }
  ros::Duration(2.0).sleep();

  return plan_success;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pregrasp_pose_publisher");
    ros::NodeHandle n;

    moveit::planning_interface::MoveGroupInterface move_group("right_arm");
    shadow_planning::PlanningOptions default_options;
    std::string ref_frame = "world";
    std::string end_effector_name = "";
    moveit::planning_interface::MoveGroupInterface::Plan output_plan;
    geometry_msgs::Pose first_pose;
    first_pose.position.x = 0.5;
    first_pose.position.y = 0.1;
    first_pose.position.z = 1.0;

    bool plan_status = false;
    plan_status = planToPoseTarget(default_options,move_group,first_pose,ref_frame,output_plan,end_effector_name);
    if (plan_status)
    {
        move_group.execute(output_plan);
    }
    else
    {
        ROS_INFO("couldn't plan a path");
    }

}