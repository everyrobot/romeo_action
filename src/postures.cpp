// MoveIt!
#include <moveit/move_group_interface/move_group.h>

#include <romeo_pick_place/postures.hpp>

Posture::Posture()
{
  double joints_head[] = {0.0, 0.4, 0.0, 0.0};
  pose_head_down = std::vector<double>(joints_head, joints_head + sizeof(joints_head) / sizeof(double) );

  double joints_arm_init[] = {1.7, 0.3, -1.3, 0.0, 0.0, 0.0};
  //double joints_arm_init[] = {1.75, 0.0, -1.0, -0.17, -0.35, 0.0};
  pose_arm_left_init = std::vector<double>(joints_arm_init, joints_arm_init + sizeof(joints_arm_init) / sizeof(double) );

  pose_arm_right_init = std::vector<double>(joints_arm_init, joints_arm_init + sizeof(joints_arm_init) / sizeof(double) );
  pose_arm_right_init[1] *= -1;
  pose_arm_right_init[2] *= -1;
  pose_arm_right_init[3] *= -1;
  pose_arm_right_init[4] *= -1;

  pose_arm_zero = std::vector<double>(6, 0.0);
  pose_hand_zero = std::vector<double>(14, 0.8);
  pose_hand_zero[0] = 0.0;

  double joints_arm_pregrasp[] = {0.8, 0.5, -0.62, -1.0, -0.96, -0.12}; //, 0.0, 0.80
  pose_arm_left_pregrasp = std::vector<double>(joints_arm_pregrasp, joints_arm_pregrasp + sizeof(joints_arm_pregrasp) / sizeof(double) );
}

bool Posture::poseHeadDown(){
  goToPose("head", &pose_head_down);
}

bool Posture::poseHandZero(const std::string end_eff, const std::string group)
{
  goToPose(end_eff, &pose_hand_zero);
  return goToPose(group, &pose_arm_zero);
}

bool Posture::poseHandInit(const std::string &end_eff, const std::string &group, const std::string &arm)
{
  //end-effector only
  goToPose(end_eff, &pose_hand_zero);

  bool success = false;
  if (arm == "right")
    success = goToPose(group, &pose_arm_right_init);
  else
    success = goToPose(group, &pose_arm_left_init);
  return success;
}

bool Posture::poseHandPregrasp(const std::string &end_eff, const std::string &group, const std::string &arm)
{
  //end-effector only
  goToPose(end_eff, &pose_hand_zero);

  bool success = false;
  /*if (arm == "right")
    success = goToPose(group, &pose_arm_right_init);
  else*/
    success = goToPose(group, &pose_arm_left_pregrasp);
  return success;
}

void Posture::getPose(const std::string group_name)
{
  moveit::planning_interface::MoveGroup move_group(group_name);

  //get the current set of joint values for the group
  std::vector<double> joints;
  move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joints);
  for (int i=0; i< joints.size(); ++i)
    std::cout << joints[i] << std::endl;
}

bool Posture::goToPose(const std::string group_name, std::vector<double> *pose)
{
  moveit::planning_interface::MoveGroup move_group(group_name);

  //get the current set of joint values for the group
  std::vector<double> joints;
  move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joints);

  //plan to the new joint space goal and visualize the plan.
  if (joints.size() == pose->size())
  {
    /*for (int i=0; i< joints.size(); ++i)
      std::cout << joints[i] << std::endl;*/

    move_group.setJointValueTarget(*pose);

    moveit::planning_interface::MoveGroup::Plan plan;
    bool success = move_group.plan(plan);
    sleep(1.0);
    if (success)
    {
      ROS_INFO("Action with the move_group %s", success?"":"FAILED");
      move_group.move();
      sleep(1.0);
    }
    return success;
  }
  else
    ROS_INFO_STREAM_NAMED("pick_place:","Input joint values have wrong size " << joints.size() << " == " << pose->size());
  return false;
}
