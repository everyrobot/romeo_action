#ifndef ACTIONS_HPP
#define ACTIONS_HPP

// ROS
#include <ros/ros.h>

#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasps

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

//#include <romeo_pick_place/custom_environment5.hpp>

#include <romeo_pick_place/metablock.hpp>

namespace romeo_pick_place
{

class Actions
{
public:
  Actions(ros::NodeHandle *nh_, moveit_visual_tools::MoveItVisualToolsPtr visual_tools_, const std::string arm_);
  bool pickAction(MetaBlock *block); //const geometry_msgs::Pose& block_pose, std::string block_name);
  bool placeAction(MetaBlock *block); //const geometry_msgs::Pose& block_pose, std::string block_name);

  bool approachAction(MetaBlock *block);
  bool pickDefault(MetaBlock *block);

  bool returnAction(const std::string group_name);

  void testReachablity(MetaBlock *block);


  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // which arm are we using
  std::string arm_;
  std::string end_effector_;
  std::string planning_group_name_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // grasp generator
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  moveit::planning_interface::MoveGroup::Plan plan;
};

}

#endif // ACTIONS_HPP
