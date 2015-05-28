#ifndef SIMPLEPICKPLACE_H
#define SIMPLEPICKPLACE_H

// ROS
#include <ros/ros.h>

#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasps

//to move the head
/*#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>*/

#include <romeo_pick_place/metablock.hpp>
#include <romeo_pick_place/actions.hpp>

namespace romeo_pick_place
{

//const std::string OBJECT_RECOGNITION_ACTION = "/recognize_objects";

class SimplePickPlace
{
public:
  SimplePickPlace(const bool verbose);
  bool startRoutine();
  void testReachablePoses();

  void resetBlock(MetaBlock block);
  void publishMetaBlock(MetaBlock *block, rviz_visual_tools::colors color, const bool goal);
  void publishCollisionMetaBlock(MetaBlock *block);

  void updateObjects(const moveit_msgs::CollisionObject::ConstPtr& msg);
  void updateObjectsVirtual();
  void cleanObjects();

  // A shared node handle
  ros::NodeHandle *nh_;

  Actions action;

  // Show more visual and console output, with general slower run time.
  const bool verbose_;
  const bool saveStat_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  std::vector<MetaBlock> blocks;

  ros::Subscriber sub_obj_coll;
  ros::Publisher pub_obj_pose, pub_obj_poses;
  geometry_msgs::PoseStamped msg_obj_pose; //, msg_obj_ork_;
  geometry_msgs::PoseArray msg_obj_poses;
  //ros::ServiceClient planning_scene_service_;

  std::vector<geometry_msgs::Pose> poses_validated;
  std::vector<geometry_msgs::Pose> poses_failed;

  //ORK
  /*ros::Subscriber sub_ork_;
  ros::Publisher pub_obj_col;
  tf::TransformListener listener_;
  std::string m_target_frame;
  std::string m_depth_frame_id;
  shape_msgs::SolidPrimitive msg_cylinder_;*/
  //std::vector <moveit_msgs::CollisionObject> msg_obj_cols;

  //to move the head
  /*robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;*/

};
}

#endif // SIMPLEPICKPLACE_H
