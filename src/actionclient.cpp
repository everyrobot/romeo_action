#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include <romeo_pick_place/toolsForAction.hpp>
#include <romeo_pick_place/actionclient.hpp>

actionClient::actionClient(ros::NodeHandle *nh_)
{

  //directly with action client

  //current_state_monitor_ = getSharedStateMonitor(robot_model_, tf_);

  ros::Duration wait_for_server = ros::Duration(5, 0);
  move_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(*nh_,
                                                                                            move_group::MOVE_ACTION,
                                                                                            false));
  try
  {
    waitForAction(move_action_client_, *nh_, wait_for_server, move_group::MOVE_ACTION);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Connecting to move_group::move action server: %s", ex.what());
    return;
  }

  pick_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(*nh_,
                                                                                         move_group::PICKUP_ACTION,
                                                                                         false));
  try
  {
    waitForAction(pick_action_client_, *nh_, wait_for_server, move_group::PICKUP_ACTION);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Connecting to move_group::pickup action server: %s", ex.what());
    return;
  }

  place_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(*nh_,
                                                                                         move_group::PLACE_ACTION,
                                                                                         false));
  try
  {
    waitForAction(place_action_client_, *nh_, wait_for_server, move_group::PLACE_ACTION);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Connecting to move_group::place action server: %s", ex.what());
    return;
  }
}

void actionClient::setStartState(const robot_state::RobotState &start_state)
{
  considered_start_state_.reset(new robot_state::RobotState(start_state));
}


/*void Action::constructGoal(moveit_msgs::MoveGroupGoal &goal_out) //, const std::string &object)
{
  moveit_msgs::MoveGroupGoal goal;
  //goal.target_name = object;
  goal.request.group_name = plan_group;
  goal.request.num_planning_attempts = 10;
  goal.request.max_velocity_scaling_factor = 1.0;
  goal.request.allowed_planning_time = 30.0;
  //goal.request.planner_id = planner_id_;
  //goal.request.workspace_parameters = workspace_parameters_;

  if (considered_start_state_)
    robot_state::robotStateToRobotStateMsg(*considered_start_state_, goal.request.start_state);

  if (active_target_ == JOINT)
  {
    goal.request.goal_constraints.resize(1);
    goal.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(getJointStateTarget(), joint_model_group_, goal_joint_tolerance_);
  }
  else
    if (active_target_ == POSE || active_target_ == POSITION || active_target_ == ORIENTATION)
    {
      // find out how many goals are specified
      std::size_t goal_count = 0;
      for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin() ;
           it != pose_targets_.end() ; ++it)
        goal_count = std::max(goal_count, it->second.size());

      // start filling the goals;
      // each end effector has a number of possible poses (K) as valid goals
      // but there could be multiple end effectors specified, so we want each end effector
      // to reach the goal that corresponds to the goals of the other end effectors
      goal.request.goal_constraints.resize(goal_count);

      for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin() ;
           it != pose_targets_.end() ; ++it)
      {
        for (std::size_t i = 0 ; i < it->second.size() ; ++i)
        {
          moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints(it->first, it->second[i], goal_position_tolerance_, goal_orientation_tolerance_);
          if (active_target_ == ORIENTATION)
            c.position_constraints.clear();
          if (active_target_ == POSITION)
            c.orientation_constraints.clear();
          goal.request.goal_constraints[i] = kinematic_constraints::mergeConstraints(goal.request.goal_constraints[i], c);
        }
      }
    }
    else
      ROS_ERROR("Unable to construct goal representation");

  if (path_constraints_)
    goal.request.path_constraints = *path_constraints_;
  goal_out = goal;
}

int Action::planEnhanced(moveit::planning_interface::MoveGroup::Plan &plan)
{
  if (!move_action_client_)
  {
    return moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  if (!move_action_client_->isServerConnected())
  {
    return moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  moveit_msgs::MoveGroupGoal goal;
  constructGoal(goal);
  goal.planning_options.plan_only = true;
  goal.planning_options.look_around = false;
  goal.planning_options.replan = false;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  move_action_client_->sendGoal(goal);
  if (!move_action_client_->waitForResult())
  {
    ROS_INFO_STREAM("MoveGroup action returned early");
  }
  if (move_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std::cout << "-- a) plan 172 SUCCEEDED"<< std::endl;
    plan.trajectory_ = move_action_client_->getResult()->planned_trajectory;
    plan.start_state_ = move_action_client_->getResult()->trajectory_start;
    plan.planning_time_ = move_action_client_->getResult()->planning_time;
    moveit_msgs::MoveItErrorCodes code = move_action_client_->getResult()->error_code;
    return int(code.val);
  }
  else
  {
    std::cout << "-- a) plan 180 FAILED" << move_action_client_->getState().toString() << ": " << move_action_client_->getState().getText() << std::endl;
    ROS_WARN_STREAM("Fail: " << move_action_client_->getState().toString() << ": " << move_action_client_->getState().getText());
    moveit_msgs::MoveItErrorCodes code = move_action_client_->getResult()->error_code;
    return int(code.val);
  }
}*/



