#ifndef SIMPLEPICKPLACE_H
#define SIMPLEPICKPLACE_H

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <romeo_pick_place/metablock.hpp>
#include <romeo_pick_place/action.hpp>
#include <romeo_pick_place/postures.hpp>
#include <romeo_pick_place/objprocessing.hpp>

namespace romeo_pick_place
{
class SimplePickPlace
{
public:
  SimplePickPlace(const bool verbose);
  bool startRoutine();

  void testReachablePoses(bool test_poses_rnd=false);
  void testReachableSingleHand(Action *action, ros::Publisher *pub_obj_poses, std::vector<MetaBlock> &blocks);
  geometry_msgs::PoseArray generatePosesRnd(const int poses_nbr, std::vector<MetaBlock> &blocks);
  geometry_msgs::PoseArray generatePosesGrid(std::vector<MetaBlock> &blocks);

  void resetBlock(MetaBlock *block);
  void publishCollisionMetaBlock(MetaBlock *block);

  void setCollisionObjects();
  void getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg);
  void updateObjectsVirtual();
  void cleanObjects();
  void removeObjects();
  moveit_msgs::CollisionObject wrapToCollisionObject(MetaBlock *block);
  bool getMeshFromDatabasePose(object_recognition_msgs::GetObjectInformation &obj_info);

  // A shared node handle
  ros::NodeHandle nh_, nh_priv_;

  // Show more visual and console output, with general slower run time.
  const bool verbose_;
  const bool saveStat_;
  const int actions_nbr;
  std::string base_frame;

  Action *action_left, *action_right;
  Posture posture;
  Objprocessing objproc;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  std::vector<MetaBlock> blocks;
  std::vector<MetaBlock> blocks_test;
  //std::vector<moveit_msgs::CollisionObject> obj_coll;

  ros::Subscriber sub_obj_coll;
  ros::Publisher pub_obj_poses, pub_obj_coll, pub_obj_pose;
  geometry_msgs::PoseStamped msg_obj_pose;
  geometry_msgs::PoseArray msg_obj_poses;
  //ros::ServiceClient planning_scene_service_;

  std::vector<geometry_msgs::Pose> poses_validated;
  std::vector<geometry_msgs::Pose> poses_failed;
};
}

#endif // SIMPLEPICKPLACE_H
