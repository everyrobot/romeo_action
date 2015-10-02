#include "romeo_pick_place/grasp_filter.hpp"

namespace romeo_pick_place
{

GraspFilter::GraspFilter(/* const std::string& base_link, bool rviz_verbose,
                          //VisualizationToolsPtr rviz_tools,*/
                          const std::string& planning_group):
  base_link_("odom"),//base_link),
  /*rviz_verbose_(rviz_verbose),
  //rviz_tools_(rviz_tools),*/
  planning_group_(planning_group),
  move_group_eef(moveit::planning_interface::MoveGroup(planning_group_))
{
  ROS_INFO_STREAM_NAMED("grasp","GraspFilter ready.");

  // Get the planning
  robot_model_ = move_group_eef.getCurrentState()->getRobotModel();
}

// Return grasps that are kinematically feasible
bool GraspFilter::filterGrasps(std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // -----------------------------------------------------------------------------------------------
  // Error check
  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("grasp","Unable to filter grasps because vector is empty");
    return false;
  }

  // -----------------------------------------------------------------------------------------------
  // how many cores does this computer have and how many do we need?
  int num_threads = boost::thread::hardware_concurrency();
  if( num_threads > possible_grasps.size() )
    num_threads = possible_grasps.size();

  if(true)
  {
    num_threads = 1;
    ROS_ERROR_STREAM_NAMED("grasp_filter","Using " << num_threads << " threads");
  }

  // -----------------------------------------------------------------------------------------------
  // Get the solver timeout from kinematics.yaml
  double timeout = robot_model_->getJointModelGroup( planning_group_ )->getDefaultIKTimeout();
  ROS_INFO_STREAM_NAMED("grasp_filter","--Planning timeout " << timeout << " == "
                        << move_group_eef.getCurrentState()->getRobotModel()->getJointModelGroup(planning_group_)->getDefaultIKTimeout());
  timeout = 0.05;

  // -----------------------------------------------------------------------------------------------
  // Load kinematic solvers if not already loaded
  if( kin_solvers_.size() != num_threads )
  {
    kin_solvers_.clear();

    boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> kin_plugin_loader;
    kin_plugin_loader.reset(new kinematics_plugin_loader::KinematicsPluginLoader());
    robot_model::SolverAllocatorFn kin_allocator = kin_plugin_loader->getLoaderFunction();

    const robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(planning_group_);
  //ROS_INFO_STREAM_NAMED("grasp_filter","planning_group_ = " << kin_plugin_loader-);
    // Create an ik solver for every thread
    for (int i = 0; i < num_threads; ++i)
    {
      ROS_INFO_STREAM_NAMED("grasp","Creating ik solver " << i);

      kin_solvers_.push_back(kin_allocator(joint_model_group));

      // Test to make sure we have a valid kinematics solver
      if( !kin_solvers_[i] )
      {
        ROS_ERROR_STREAM_NAMED("grasp_filter","No kinematic solver found");
        return false;
      }
    }
  }

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();
  {

    // -----------------------------------------------------------------------------------------------
    // Loop through poses and find those that are kinematically feasible
    std::vector<moveit_msgs::Grasp> filtered_grasps;

    boost::thread_group bgroup; // create a group of threads
    boost::mutex lock; // used for sharing the same data structures

    ROS_INFO_STREAM_NAMED("grasp", "Filtering possible grasps with " << num_threads << " threads");

    // split up the work between threads
    double num_grasps_per_thread = double(possible_grasps.size()) / num_threads;
    //ROS_INFO_STREAM("total grasps " << possible_grasps.size() << " per thead: " << num_grasps_per_thread);

    int grasps_id_start;
    int grasps_id_end = 0;

    for(int i = 0; i < num_threads; ++i)
    {
      grasps_id_start = grasps_id_end;
      grasps_id_end = ceil(num_grasps_per_thread*(i+1));
      if( grasps_id_end >= possible_grasps.size() )
        grasps_id_end = possible_grasps.size();
      //ROS_INFO_STREAM_NAMED("grasp","low " << grasps_id_start << " high " << grasps_id_end);

      IkThreadStruct tc(possible_grasps, filtered_grasps, grasps_id_start, grasps_id_end,
                        kin_solvers_[i], timeout, &lock, i);
      bgroup.create_thread( boost::bind( &GraspFilter::filterGraspThread, this, tc ) );
    }

    ROS_INFO_STREAM_NAMED("grasp","Waiting to joint threads...");
    bgroup.join_all(); // wait for all threads to finish
    ROS_INFO_STREAM_NAMED("grasp","Done waiting to joint threads...");

    ROS_INFO_STREAM_NAMED("grasp", "Found " << filtered_grasps.size() << " ik solutions out of " <<
                          possible_grasps.size() );

    possible_grasps = filtered_grasps;

  }
  // End Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("grasp","Grasp generator IK grasp filtering benchmark time:");
  std::cout << duration << "\t" << possible_grasps.size() << "\n";

  ROS_INFO_STREAM_NAMED("grasp","Possible grasps filtered to " << possible_grasps.size() << " options.");

  return true;
}

// Thread for checking part of the possible grasps list
void GraspFilter::filterGraspThread(IkThreadStruct ik_thread_struct)
{
  // Seed state - start at zero
  std::vector<double> ik_seed_state; //(move_group_eef.getActiveJoints().size());//5); //7); // fill with zeros
  move_group_eef.getCurrentState()->copyJointGroupPositions(move_group_eef.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_eef.getName()), ik_seed_state);

  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  geometry_msgs::Pose* ik_pose;

  // Process the assigned grasps
  for( int i = ik_thread_struct.grasps_id_start_; i < ik_thread_struct.grasps_id_end_; ++i )
  {
    ROS_DEBUG_STREAM_NAMED("grasp", "Checking grasp #" << i);

    // Pointer to current pose
    ik_pose = &ik_thread_struct.possible_grasps_[i].grasp_pose.pose;


    ROS_WARN_STREAM_NAMED("temp","ik_pose" << *ik_pose);
    std::copy(ik_seed_state.begin(), ik_seed_state.end(), std::ostream_iterator<double>(std::cout, "\n"));
    ROS_WARN_STREAM_NAMED("temp","timeout" << ik_thread_struct.timeout_);
    //std::copy(solution.begin(), solution.end(), std::ostream_iterator<double>(std::cout, "\n"));
    ROS_WARN_STREAM_NAMED("temp","error_code" << error_code);

    kinematics::KinematicsQueryOptions options;
    //options.return_approximate_solution = true;

    // Test it with IK
    ik_thread_struct.kin_solver_->
      searchPositionIK(*ik_pose, ik_seed_state, ik_thread_struct.timeout_, solution, error_code, options);

    // Results
    if( error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS )
    {
      ROS_INFO_STREAM_NAMED("grasp","Found IK Solution");

      // Copy solution to seed state so that next solution is faster
      ik_seed_state = solution;

      // Copy solution to manipulation_msg so that we can use it later
      // Note: doesn't actually belong here TODO: fix this hack
      //ik_thread_struct.possible_grasps_[i].grasp_posture.position = solution;

      // Lock the result vector so we can add to it for a second
      {
        boost::mutex::scoped_lock slock(*ik_thread_struct.lock_);
        ik_thread_struct.filtered_grasps_.push_back( ik_thread_struct.possible_grasps_[i] );
      }

      // TODO: is this thread safe? (prob not)
      /*if(rviz_verbose_)
        rviz_tools_->publishArrow(*ik_pose);*/
    }
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
      ROS_INFO_STREAM_NAMED("grasp","Unable to find IK solution for pose.");
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
    {
      ROS_INFO_STREAM_NAMED("grasp","Unable to find IK solution for pose: Timed Out.");
      //std::copy(solution.begin(),solution.end(), std::ostream_iterator<double>(std::cout, "\n"));
    }
    else
      ROS_INFO_STREAM_NAMED("grasp","IK solution error: MoveItErrorCodes.msg = " << error_code);
  }

  ROS_INFO_STREAM_NAMED("grasp","Thread " << ik_thread_struct.thread_id_ << " finished");
}

}

