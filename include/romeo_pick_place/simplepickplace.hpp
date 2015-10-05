#ifndef SIMPLEPICKPLACE_H
#define SIMPLEPICKPLACE_H

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <romeo_pick_place/metablock.hpp>
#include <romeo_pick_place/action.hpp>
//#include <romeo_pick_place/postures.hpp>
#include <romeo_pick_place/objprocessing.hpp>

namespace romeo_pick_place
{

// block dimensions

class SimplePickPlace
{
public:
  SimplePickPlace(const std::string robot_name, const double test_step, const bool verbose);
  bool startRoutine();

  void testReachablePoses(const bool pickVsReach=true, bool test_poses_rnd=false);
  void testReachableSingleHand(Action *action, ros::Publisher *pub_obj_poses, std::vector<MetaBlock> &blocks, const bool pickVsReach);
  geometry_msgs::PoseArray generatePosesRnd(const int poses_nbr, std::vector<MetaBlock> &blocks);
  geometry_msgs::PoseArray generatePosesGrid(std::vector<MetaBlock> &blocks);

  void resetBlock(MetaBlock *block);
  void publishCollisionMetaBlock(MetaBlock *block);

  void setCollisionObjects();
  void getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg);
  void cleanObjects();
  void removeObjects();
  moveit_msgs::CollisionObject wrapToCollisionObject(MetaBlock *block);

  // A shared node handle
  ros::NodeHandle nh_, nh_priv_;

  std::string robot_name_;
  double test_step_;
  const bool verbose_;
  const bool saveStat_;
  std::string base_frame;
  double block_size;
  double block_size_r;

  bool env_shown_;
  double x_min;
  double x_max;
  double z_min;
  double z_max;

  Action *action_left, *action_right;
  //Posture posture;
  Objprocessing objproc;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  std::vector<MetaBlock> blocks;
  std::vector<MetaBlock> blocks_test;
  //std::vector<moveit_msgs::CollisionObject> obj_coll;

  ros::Subscriber sub_obj_coll;
  ros::Publisher pub_obj_poses, pub_obj_pose; //pub_obj_coll
  geometry_msgs::PoseStamped msg_obj_pose;
  geometry_msgs::PoseArray msg_obj_poses;
  //ros::ServiceClient planning_scene_service_;

  std::vector<geometry_msgs::Pose> poses_validated;
  std::vector<geometry_msgs::Pose> poses_failed;

  geometry_msgs::Pose pose_default, pose_default_r;

  std::vector <geometry_msgs::Pose> stat_poses_success;
};
}

#endif // SIMPLEPICKPLACE_H
