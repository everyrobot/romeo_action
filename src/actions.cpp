#include <romeo_pick_place/actions.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>


namespace romeo_pick_place
{

Actions::Actions(ros::NodeHandle *nh_, moveit_visual_tools::MoveItVisualToolsPtr visual_tools_, const std::string arm_):
  visual_tools_(visual_tools_),
  arm_()
{
  end_effector_ = arm_+"_hand";
  planning_group_name_ = arm_+"_arm";
  ROS_INFO_STREAM_NAMED("test","Arm: " << arm_);
  ROS_INFO_STREAM_NAMED("test","End Effector: " << end_effector_);
  ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);

  // Create MoveGroup for one of the planning groups
  move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
  move_group_->setPlanningTime(30.0);

  // Load grasp generator
  if (!grasp_data_.loadRobotGraspData(*nh_, end_effector_))
    ros::shutdown();

  //ROS_WARN_STREAM_NAMED("temp","temp modifications, move this maybe");
  grasp_data_.grasp_depth_ = 0.0;

  //update visualization
  visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);

  // Load Grasp generator
  simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));
}

void Actions::testReachablity(MetaBlock *block)
{
  //test the pick
  if(!pickAction(block))
  {
    //poses_failed.push_back(block->start_pose);
  }
  else
  {
    //poses_validated.push_back(block->start_pose);
    sleep(1.0);

    //return the arm
    if(returnAction(planning_group_name_))
      sleep(1.0);
  }

  //remove the block from the seen
  //removeBlock(block);
}


bool Actions::approachAction(MetaBlock *block)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  /*ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;*/

  ROS_INFO("Reference frame: %s", move_group_->getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", move_group_->getEndEffectorLink().c_str());

  /*moveit::planning_interface::MotionPlanRequest req;
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(move_group_->getEndEffectorLink(), block_pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);*/

  geometry_msgs::PoseStamped block_pose1;
  block_pose1.header.frame_id = "base_link";
  /*block_pose1.pose.position.x = 0.5;
  block_pose1.pose.position.y = 0.2;
  block_pose1.pose.position.z = -0.13;*/
  block_pose1.pose.position.x = 0.419093;
  block_pose1.pose.position.y = 0.2;
  block_pose1.pose.position.z = -0.0640153;
  block_pose1.pose.orientation.x = 0.000000199;
  block_pose1.pose.orientation.y = 0.195092;
  block_pose1.pose.orientation.z = -0.000000199;
  block_pose1.pose.orientation.w = 0.980785;

  ROS_INFO("Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(block->name);
  planning_scene_interface.removeCollisionObjects(object_ids);

  move_group_->setStartState(*move_group_->getCurrentState());
  //move_group_->setPoseReferenceFrame("base_link");
  move_group_->setPoseTarget(block_pose1, "LWristYaw_link");

  //call the planner to compute the plan and visualize it
  bool success = move_group_->plan(plan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  //sleep(5.0);

    /*ROS_INFO("Visualizing plan 1 (again)");
    display_trajectory.trajectory_start = plan.start_state_;
    display_trajectory.trajectory.push_back(plan.trajectory_);
    display_publisher.publish(display_trajectory);
    //Sleep to give Rviz time to visualize the plan.
    sleep(5.0);*/

  if (success)
  {
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(1.0);
    move_group_->move();
    sleep(1.0);
  }
  return false;
}

bool Actions::pickDefault(MetaBlock *block)
{
  std::vector<moveit_msgs::Grasp> grasps(1);
  bool done = false;
  moveit_msgs::Grasp g;

  /*std::vector<moveit_msgs::Grasp> grasps_gen;
  simple_grasps_->generateBlockGrasps( block_pose, grasp_data_, grasps_gen );

 for (int i=0; i<grasps_gen.size(); ++i)
  {
    g = grasps_gen[i];*/

g.grasp_pose.header.frame_id = "base_link";
g.grasp_pose.pose = block->start_pose;
g.grasp_pose.pose.position.x = 0.419093;
g.grasp_pose.pose.position.y = 0.2;
g.grasp_pose.pose.position.z = -0.0640153;
g.grasp_pose.pose.orientation.x = 0.000000199;
g.grasp_pose.pose.orientation.y = 0.195092;
g.grasp_pose.pose.orientation.z = -0.000000199;
g.grasp_pose.pose.orientation.w = 0.980785;

g.pre_grasp_approach.direction.header.frame_id = "base_link"; //"LWristYaw_link"; //lscene->getPlanningFrame();
g.pre_grasp_approach.direction.vector.x = 0;
g.pre_grasp_approach.direction.vector.y = 0;
g.pre_grasp_approach.direction.vector.z = -1;
g.pre_grasp_approach.min_distance = 0.06;
g.pre_grasp_approach.desired_distance = 0.2;

g.post_grasp_retreat.direction.header.frame_id = "base_link"; //"LWristYaw_link"; //lscene->getPlanningFrame();
g.post_grasp_retreat.direction.vector.x = 0;
g.post_grasp_retreat.direction.vector.y = 0;
g.post_grasp_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
g.post_grasp_retreat.min_distance = 0.06;
g.post_grasp_retreat.desired_distance = 0.2;

g.pre_grasp_posture.header.frame_id = "base_link";
g.pre_grasp_posture.joint_names.resize(1);
g.pre_grasp_posture.joint_names[0] = "LHand";
g.pre_grasp_posture.points.resize(1);
g.pre_grasp_posture.points[0].positions.resize(1);
g.pre_grasp_posture.points[0].positions[0] = 0.0;

g.grasp_posture.header.frame_id = "base_link";
g.grasp_posture.joint_names.resize(1);
g.grasp_posture.joint_names[0] = "LHand";
g.grasp_posture.points.resize(1);
g.grasp_posture.points[0].positions.resize(1);
g.grasp_posture.points[0].positions[0] = 1.0;

std::cout << "-- pickDefault g " << g << std::endl;

    grasps[0] = g;

    /*if (verbose_)
    {
      double speed = 0.012; //0.05; //
      visual_tools_->publishGrasps( grasps, grasp_data_.ee_parent_link_, speed);
      visual_tools_->deleteAllMarkers();
    }*/

    // Prevent collision with table
    //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects;
    /*std::vector<MetaBlock>::iterator blocks_i;
    for (blocks_i=blocks.begin(); blocks_i!=blocks.end(); ++blocks_i)
      allowed_touch_objects.push_back(blocks_i->name);*/
    allowed_touch_objects.push_back(block->name);

    // Add this list to all grasps
    for (std::size_t i = 0; i < grasps.size(); ++i)
      grasps[i].allowed_touch_objects = allowed_touch_objects;

    //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << grasps[0]);
    if (move_group_->pick(block->name, grasps)){
      done = true;

      std::cout << "-- pickDefault g " << g << std::endl;
      /*std::cout << "-- grasp_pose " << g.grasp_pose << std::endl;
      std::cout << "-- pre_grasp_approach " << g.pre_grasp_approach << std::endl;
      std::cout << "-- post_grasp_retreat " << g.post_grasp_retreat << std::endl;
      std::cout << "-- grasp_quality " << g.grasp_quality << std::endl;
      std::cout << "-- pre_grasp_posture " << g.pre_grasp_posture << std::endl;
      std::cout << "-- grasp_posture " << g.grasp_posture << std::endl;
      std::cout << "-- max_contact_force " << g.max_contact_force << std::endl;*/

      return true;
    }
    sleep(0.7);
  //}
}

bool Actions::pickAction(MetaBlock *block)
{
  ROS_INFO_STREAM_NAMED("pick_place:","Picking '" << block->name << "'");
  std::cout << "at pose " << block->start_pose << std::endl;

  // Visualize the block we are about to pick
  /*if (verbose_)
  {
    visual_tools_->deleteAllMarkers();
    publishMetaBlock(block, rviz_visual_tools::PURPLE, 0);
  }*/

  std::vector<moveit_msgs::Grasp> grasps;
  simple_grasps_->generateBlockGrasps(block->start_pose, grasp_data_, grasps );
  std::cout << " -- pickAction grasps.size()=" << grasps.size() << std::endl;

  //if (verbose_)
  {
    double speed = 0.012; //0.05; //
    visual_tools_->publishGrasps( grasps, grasp_data_.ee_parent_link_, speed);
    visual_tools_->deleteAllMarkers();
  }

  // Prevent collision with table
  //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

  // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
  std::vector<std::string> allowed_touch_objects;
  /*std::vector<MetaBlock>::iterator blocks_i;
  for (blocks_i=blocks.begin(); blocks_i!=blocks.end(); ++blocks_i)
    allowed_touch_objects.push_back(blocks_i->name);*/
  allowed_touch_objects.push_back(block->name);

  // Add this list to all grasps
  for (std::size_t i = 0; i < grasps.size(); ++i)
    grasps[i].allowed_touch_objects = allowed_touch_objects;

  //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << grasps[0]);
  return move_group_->pick(block->name, grasps);
}

bool Actions::placeAction(MetaBlock *block)
{
  ROS_INFO_STREAM_NAMED("pick_place:","Placing '" << block->name << "'");
  std::cout << "at pose " << block->goal_pose << std::endl;

  // Visualize the goal block location
  /*if (verbose_)
  {
    visual_tools_->deleteAllMarkers();
    publishMetaBlock(block, rviz_visual_tools::PURPLE, 1);
  }*/

  /*if (verbose_)
  {
    msg_obj_pose.pose = block_pose;
    pub_obj_pose.publish(msg_obj_pose);
  }*/

  std::vector<moveit_msgs::PlaceLocation> place_locations;

  // Re-usable datastruct
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = grasp_data_.base_link_;
  pose_stamped.header.stamp = ros::Time::now();

  // Create 360 degrees of place location rotated around a center
  //for (double angle = 0; angle < 2*M_PI; angle += M_PI/2)
  {
    pose_stamped.pose = block->goal_pose;

    // Create new place location
    moveit_msgs::PlaceLocation place_loc;

    place_loc.place_pose = pose_stamped;

    // Approach
    moveit_msgs::GripperTranslation pre_place_approach;
    pre_place_approach.direction.header.stamp = ros::Time::now();
    pre_place_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
    pre_place_approach.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
    pre_place_approach.direction.header.frame_id = grasp_data_.base_link_;
    std::cout << grasp_data_.base_link_ << std::endl;
    pre_place_approach.direction.vector.x = 0;
    pre_place_approach.direction.vector.y = 0;
    pre_place_approach.direction.vector.z = 0.1; //-1 // Approach direction (negative z axis)  // TODO: document this assumption
    place_loc.pre_place_approach = pre_place_approach;

    // Retreat
    /*moveit_msgs::GripperTranslation post_place_retreat(pre_place_approach);
    post_place_retreat.direction.vector.x = 0;
    post_place_retreat.direction.vector.y = 0;
    post_place_retreat.direction.vector.z = -1; //1 // Retreat direction (pos z axis)
    place_loc.post_place_retreat = post_place_retreat;*/
    // Post place posture - use same as pre-grasp posture (the OPEN command)
    place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

    place_locations.push_back(place_loc);
  }

  // Prevent collision with table
  //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME); //UNCOMMENT !!!!

  move_group_->setPlannerId("RRTConnectkConfigDefault");

  return move_group_->place(block->name, place_locations);
}

bool Actions::returnAction(const std::string group_name)
{
  std::vector<double> joints_arm;
  //reset the hand to intial pose
  moveit::planning_interface::MoveGroup move_group_hand(group_name);
  joints_arm.clear();
  //get the current set of joint values for the group.
  move_group_hand.getCurrentState()->copyJointGroupPositions(move_group_hand.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_hand.getName()), joints_arm);
  //plan to the new joint space goal and visualize the plan.
  move_group_hand.setJointValueTarget(joints_arm);

  moveit::planning_interface::MoveGroup::Plan plan_pose_arm;
  bool success = move_group_hand.plan(plan_pose_arm);
  sleep(1.0);
  if (success)
  {
    ROS_INFO("Action %s",success?"":"FAILED");
    move_group_hand.move();
    sleep(1.0);
  }
  return success;
}
}
