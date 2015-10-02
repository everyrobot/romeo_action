#ifndef GRASP_FILTER_HPP
#define GRASP_FILTER_HPP

#include <moveit_msgs/Grasp.h>

// MoveIt
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>

namespace romeo_pick_place
{

// Struct for passing parameters to threads, for cleaner code
struct IkThreadStruct
{
  IkThreadStruct(std::vector<moveit_msgs::Grasp> &possible_grasps, // the input
                 std::vector<moveit_msgs::Grasp> &filtered_grasps, // the result
                 int grasps_id_start,
                 int grasps_id_end,
                 kinematics::KinematicsBasePtr kin_solver,
                 double timeout,
                 boost::mutex *lock,
                 int thread_id)
    : possible_grasps_(possible_grasps),
      filtered_grasps_(filtered_grasps),
      grasps_id_start_(grasps_id_start),
      grasps_id_end_(grasps_id_end),
      kin_solver_(kin_solver),
      timeout_(timeout),
      lock_(lock),
      thread_id_(thread_id)
  {
  }
  std::vector<moveit_msgs::Grasp> &possible_grasps_;
  std::vector<moveit_msgs::Grasp> &filtered_grasps_;
  int grasps_id_start_;
  int grasps_id_end_;
  kinematics::KinematicsBasePtr kin_solver_;
  double timeout_;
  boost::mutex *lock_;
  int thread_id_;
};

class GraspFilter
{
public:
  GraspFilter(const std::string& planning_group);

  // Thread for checking part of the possible grasps list
  void filterGraspThread(IkThreadStruct ik_thread_struct);

  bool filterGrasps(std::vector<moveit_msgs::Grasp>& possible_grasps);

private:

  // Shared robot model
  robot_model::RobotModelConstPtr robot_model_;

  const std::string base_link_;
  const std::string planning_group_;
  moveit::planning_interface::MoveGroup move_group_eef;

  // threaded kinematic solvers
  std::vector<kinematics::KinematicsBasePtr> kin_solvers_;

  // whether to publish grasp info to rviz
  bool rviz_verbose_;

  // class for publishing stuff to rviz
  //block_grasp_generator::VisualizationToolsPtr rviz_tools_;

};

//typedef boost::shared_ptr<GraspFilter> GraspFilterPtr;

}

#endif // GRASP_FILTER_HPP
