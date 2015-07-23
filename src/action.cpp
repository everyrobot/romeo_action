#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <romeo_pick_place/action.hpp>

namespace romeo_pick_place
{

Action::Action(ros::NodeHandle *nh_, moveit_visual_tools::MoveItVisualToolsPtr &visual_tools, const std::string arm):
  verbose_(true),
  arm(arm)
{
  end_eff = arm+"_hand";
  plan_group = arm+"_arm";
  /*ROS_INFO_STREAM_NAMED("test","Arm: " << arm);
  ROS_INFO_STREAM_NAMED("test","End Effector: " << end_eff);
  ROS_INFO_STREAM_NAMED("test","Planning Group: " << plan_group);*/

  // Create MoveGroup for one of the planning groups
  move_group_.reset(new move_group_interface::MoveGroup(plan_group));
  move_group_->setPlanningTime(30.0);
  //should we remove it? //we should add stiffness !!!
  move_group_->setGoalPositionTolerance(0.01); //(0.001);
  move_group_->setGoalOrientationTolerance(0.1);//(0.01);
  //move_group_->setNumPlanningAttempts
/*std::cout << "-------- move_group_->getGoalPositionTolerance()= " << move_group_->getGoalPositionTolerance() << std::endl;
std::cout << "-------- move_group_->getGoalOrientationTolerance()= " << move_group_->getGoalOrientationTolerance() << std::endl;*/

  // Load grasp generator
  if (!grasp_data_.loadRobotGraspData(*nh_, end_eff))
  {
    ROS_ERROR("The grasp data cannot be loaded");
    ros::shutdown();
  }

  //ROS_WARN_STREAM_NAMED("temp","temp modifications, move this maybe");
  grasp_data_.grasp_depth_ = 0.0;

  visual_tools_ = visual_tools;

  //update visualization
  visual_tools_->loadEEMarker(grasp_data_.ee_group_, plan_group);

  // Load Grasp generator
  simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));
}

bool Action::approachAction(MetaBlock *block)
{
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
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
  block_pose1.header.frame_id = grasp_data_.base_link_;//"base_link";
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
  //planning_scene_interface.removeCollisionObjects(object_ids);

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

/*bool Action::pickDefault(MetaBlock *block)
{
  std::vector<moveit_msgs::Grasp> grasps(1);
  bool done = false;
  moveit_msgs::Grasp g;

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

    // Prevent collision with table
    //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(block->name);

    // Add this list to all grasps
    for (std::size_t i = 0; i < grasps.size(); ++i)
      grasps[i].allowed_touch_objects = allowed_touch_objects;

    //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << grasps[0]);
    if (move_group_->pick(block->name, grasps)){
      done = true;

      return true;
    }
    sleep(0.7);
  //}
}*/

bool Action::pickDefault(MetaBlock *block)
{
  // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
  std::vector<std::string> allowed_touch_objects(1);
  allowed_touch_objects[0] = block->name;

  std::vector<moveit_msgs::Grasp> grasps;
  geometry_msgs::PoseStamped p = move_group_->getRandomPose();
  p.pose = block->start_pose;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  std::cout << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << std::endl;

  moveit_msgs::Grasp g;
  g.grasp_pose = p;
  //g.pre_grasp_approach.direction.vector.x = 1.0;
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.direction.header = p.header;
  /*g.pre_grasp_approach.min_distance = 0.2;
  g.pre_grasp_approach.desired_distance = 0.4;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.27;*/
  g.pre_grasp_approach.min_distance = 0.06; //0.2;
  g.pre_grasp_approach.desired_distance = 0.2; //0.4;
  g.post_grasp_retreat.min_distance = 0.03; //0.1;
  g.post_grasp_retreat.desired_distance = 0.11; //0.27;

  g.pre_grasp_posture.joint_names.resize(1, "LHand");
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 0;

  g.grasp_posture.joint_names.resize(1, "LHand");
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 1;

  // Add this list to all grasps
  g.allowed_touch_objects = allowed_touch_objects;
  grasps.push_back(g);
  if (move_group_->pick(block->name, grasps))
    return true;

  return false;
}

bool Action::demoPick(MetaBlock *block)
{
  // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
  std::vector<std::string> allowed_touch_objects(1);
  allowed_touch_objects[0] = block->name;

  std::vector<moveit_msgs::Grasp> grasps;
  for (std::size_t i = 0 ; i < 100; ++i)
  {
    grasps.clear();
    geometry_msgs::PoseStamped p = move_group_->getRandomPose();
    //p.pose = block->start_pose;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    std::cout << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << std::endl;

    moveit_msgs::Grasp g;
    g.grasp_pose = p;
    //g.pre_grasp_approach.direction.vector.x = 1.0;
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.direction.header = p.header;
    /*g.pre_grasp_approach.min_distance = 0.06; //0.2;
    g.pre_grasp_approach.desired_distance = 0.2; //0.4;
    g.post_grasp_retreat.min_distance = 0.06; //0.1;
    g.post_grasp_retreat.desired_distance = 0.2; //0.27;*/
    g.pre_grasp_approach.min_distance = 0.2;
    g.pre_grasp_approach.desired_distance = 0.4;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.27;

    /*g.pre_grasp_approach.min_distance = 0.1 + 0.01*(float)i;
    g.pre_grasp_approach.desired_distance = 0.3 + 0.01*(float)i;
    g.post_grasp_retreat.min_distance = 0.0 + 0.01*(float)i;
    g.post_grasp_retreat.desired_distance = 0.17 + 0.01*(float)i;*/

    g.pre_grasp_posture.joint_names.resize(1, "LHand");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(1);
    g.pre_grasp_posture.points[0].positions[0] = 0;

    g.grasp_posture.joint_names.resize(1, "LHand");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(1);
    g.grasp_posture.points[0].positions[0] = 1;

    // Add this list to all grasps
    g.allowed_touch_objects = allowed_touch_objects;
    grasps.push_back(g);
    move_group_->pick(block->name, grasps);
    /*if (move_group_->pick(block->name, grasps))
      return true;*/
  }
  return false;
}

bool Action::pickAction(MetaBlock *block, const std::string surface_name)
{
  if (verbose_)
    ROS_INFO_STREAM_NAMED("pick_place:","Picking " << block->name << " at pose " << block->start_pose);

  /*if (verbose_)
  {
    visual_tools_->deleteAllMarkers();
  }*/

  std::vector<moveit_msgs::Grasp> grasps;
  simple_grasps_->generateBlockGrasps(block->start_pose, grasp_data_, grasps );

  if (verbose_)
  {
    double speed = 0.01; //0.05; //
    visual_tools_->publishGrasps( grasps, grasp_data_.ee_parent_link_, speed);
    visual_tools_->deleteAllMarkers();
  }
  sleep(0.5);

  bool success = false;
  if (grasps.size() > 0)
  {
    // Prevent collision with table
    if (!surface_name.empty())
      move_group_->setSupportSurfaceName(surface_name);

    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects(1);
    allowed_touch_objects[0] = block->name;
    // Add this list to all grasps
    for (std::size_t i = 0; i < grasps.size(); ++i)
      grasps[i].allowed_touch_objects = allowed_touch_objects;

    success = move_group_->pick(block->name, grasps);
    if (verbose_)
    {
      if (success)
        ROS_INFO_STREAM_NAMED("pick_place","Pick success! \n\n");
      else
        ROS_ERROR_STREAM_NAMED("pick_place","Pick failed.");
    }
  }
  return success;
}

bool Action::placeAction(MetaBlock *block)
{
  ROS_INFO_STREAM_NAMED("pick_place:","Placing " << block->name << "at pose " << block->goal_pose);

  // Visualize the goal block location
  /*if (verbose_)
  {
    visual_tools_->deleteAllMarkers();
    publishMetaBlock(block, rviz_visual_tools::PURPLE, 1);
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
    ROS_INFO_STREAM_NAMED("pick_place:","place_loc.post_place_retreat" << place_loc.post_place_retreat);
    place_loc.post_place_posture = grasp_data_.pre_grasp_posture_; //grasp_data_.grasp_posture_;

    place_locations.push_back(place_loc);
  }

  // Prevent collision with table
  //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME); //UNCOMMENT !!!!

  move_group_->setPlannerId("RRTConnectkConfigDefault");

  bool success = move_group_->place(block->name, place_locations);
  if (success)
  {
    ROS_INFO_STREAM_NAMED("pick_place","Place success! \n\n");
    sleep(1.0);
  }
  else
    ROS_ERROR_STREAM_NAMED("pick_place","Place failed.");

  return success;
}

bool Action::demoPlace(MetaBlock *block)
{
  std::vector<moveit_msgs::PlaceLocation> loc;
  for (std::size_t i = 0 ; i < 20; ++i)
  {
    geometry_msgs::PoseStamped p = move_group_->getRandomPose();
    //p.pose = block->goal_pose;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    std::cout << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << std::endl;

    moveit_msgs::PlaceLocation g;
    g.place_pose = p;
    g.pre_place_approach.direction.vector.x = 1.0; //?
    g.post_place_retreat.direction.vector.z = 1.0;
    g.post_place_retreat.direction.header = p.header;
    g.pre_place_approach.min_distance = 0.2;
    g.pre_place_approach.desired_distance = 0.4;
    g.post_place_retreat.min_distance = 0.1;
    g.post_place_retreat.desired_distance = 0.27;

    g.post_place_posture.joint_names.resize(1, "LHand");
    g.post_place_posture.points.resize(1);
    g.post_place_posture.points[0].positions.resize(1);
    g.post_place_posture.points[0].positions[0] = 1;

    loc.push_back(g);

    move_group_->place(block->name, loc);
  }

  return false;
}
}
