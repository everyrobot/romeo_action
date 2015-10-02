#ifndef ACTIONCLIENT_HPP
#define ACTIONCLIENT_HPP

// ROS
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

#include <moveit/robot_state/robot_state.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>

class actionClient
{
public:
  actionClient(ros::NodeHandle *nh_);

  //planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > move_action_client_;
  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pick_action_client_;
  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > place_action_client_;

  robot_state::RobotStatePtr considered_start_state_;
  moveit_msgs::WorkspaceParameters workspace_parameters_;

private:
  int planEnhanced(moveit::planning_interface::MoveGroup::Plan &plan);
  void constructGoal(moveit_msgs::MoveGroupGoal &goal_out); //, const std::string &object)
  void setStartState(const robot_state::RobotState &start_state);
};

#endif // ACTIONCLIENT_HPP
