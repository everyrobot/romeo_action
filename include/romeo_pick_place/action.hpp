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

#include <romeo_pick_place/metablock.hpp>

namespace romeo_pick_place
{

class Action
{
public:
  Action(ros::NodeHandle *nh_, moveit_visual_tools::MoveItVisualToolsPtr &visual_tools, const std::string arm_);
  bool pickAction(MetaBlock *block, const std::string surface_name);
  bool placeAction(MetaBlock *block);
  bool approachAction(MetaBlock *block);
  bool pickDefault(MetaBlock *block);
  bool demoPick(MetaBlock *block);
  bool demoPlace(MetaBlock *block);

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  bool verbose_;

  // which arm are we using
  std::string arm;
  std::string end_eff;
  std::string plan_group;

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
