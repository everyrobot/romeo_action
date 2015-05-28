/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CU Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of CU Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * \brief   Simple pick place for blocks using Romeo
 * \author  Dave Coleman
 */

//publish messages with objects poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

//to move the head
//#include <moveit/robot_state/robot_state.h>

#include <romeo_pick_place/custom_environment5.hpp>
#include <romeo_pick_place/simplepickplace.hpp>

namespace romeo_pick_place
{

/**
   * @brief promptUserAction
   * @param obj_name object name to perform action
   * @param obj_count total number of objects
   * @return action to do: 0-nothing, 1-approach, 2-pick, 3-move, 4-release.*/
  int promptUserAction(const std::string obj_name, const int obj_count)
  {
    ROS_INFO_STREAM_NAMED("pick_place","What do you want me to do with the object "
                          << obj_name << " out of " << obj_count << " detected objects? "
                          << "Possible actions are: 0-nothing, 1-approach, 2-pick, 3-move, 4-release, 5-return.");
    int in;
    std::cin >> in;
    return in;
  }

  bool promptUser()
  {
    ROS_INFO_STREAM_NAMED("pick_place","Retry? (y/n)");
    char input; // used for prompting yes/no
    std::cin >> input;
    if( input == 'n' )
      return false;

    return true;
  }

  void swapPoses(geometry_msgs::Pose *pose1, geometry_msgs::Pose *pose2){
    geometry_msgs::Pose temp = *pose1;
    pose1 = pose2;
    *pose2 = temp;
  }

  SimplePickPlace::SimplePickPlace(const bool verbose)
    : nh_(new ros::NodeHandle("~")),
      action(nh_, visual_tools_, "left"),
      verbose_(verbose),
      saveStat_(true)/*,
      robot_model_loader("robot_description")*/
  {
    // objects related initialization
    pub_obj_pose = nh_->advertise<geometry_msgs::PoseStamped>("/obj_pose", 100);
    pub_obj_poses = nh_->advertise<geometry_msgs::PoseArray>("/obj_poses", 100);

    sub_obj_coll = nh_->subscribe<moveit_msgs::CollisionObject>("/collision_object", 10, &SimplePickPlace::updateObjects, this);
    /*sub_ork_ = nh_->subscribe<object_recognition_msgs::RecognizedObjectArray>("/recognized_object_array", 10, &SimplePickPlace::obj_transform, this);
    pub_obj_col = nh_->advertise<moveit_msgs::CollisionObject>("/collision_object", 1000);*/
    //planning_scene_service_ = nh_->serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    /*m_target_frame = "base_link";
    m_depth_frame_id = "CameraDepth_frame";
    msg_cylinder_.type = shape_msgs::SolidPrimitive::CYLINDER;
    msg_cylinder_.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = BLOCK_SIZE;
    msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = BLOCK_SIZE*3.0;*/

    std::string base_link = "base_link"; //grasp_data_.base_link_
    msg_obj_pose.header.frame_id = base_link;//grasp_data_.base_link_;
    msg_obj_poses.header.frame_id = base_link;//grasp_data_.base_link_;

    //----------------------------------------------------
    //move the head
    // Create a MoveGroup for the head
    moveit::planning_interface::MoveGroup move_group_head("head");

    //get the current set of joint values for the group.
    std::vector<double> joints_head;
    move_group_head.getCurrentState()->copyJointGroupPositions(move_group_head.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_head.getName()), joints_head);
    //plan to the new joint space goal and visualize the plan.
    if (joints_head.size()>1)
      if (joints_head[1] == 0.0)
        joints_head[1] = 0.3;
    move_group_head.setJointValueTarget(joints_head);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = move_group_head.plan(my_plan);
    ROS_INFO("Moving the head to look down - planning %s",success?"":"FAILED");
    sleep(1.0);
    if (success)
    {
      ROS_INFO("Moving the head to look down - action %s",success?"":"FAILED");
      move_group_head.move();
      sleep(1.0);
    }

      // Load the Robot Viz Tools for publishing to rviz
      visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(base_link));//grasp_data_.base_link_
      visual_tools_->setFloorToBaseHeight(-0.9);
      visual_tools_->deleteAllMarkers();

      // Let everything load
      ros::Duration(1.0).sleep();

      // Show grasp visualizations or not
      visual_tools_->setMuted(false);

      // Create the walls and tables
      createEnvironment(visual_tools_);

      //reset hand to the initial position
      action.returnAction(action.planning_group_name_);
      action.returnAction(action.end_effector_);

      //updateObjectsVirtual();

      while(ros::ok())
        startRoutine();
  }

  void SimplePickPlace::testReachablePoses()
  {
    MetaBlock *block = new MetaBlock("BlockTest", 0.5, 0.2, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE);

    int count = 0;
    while (count < 100){
      //generate a random position
      //block->applyRndPose();

      action.testReachablity(block);
      ++count;
    }
  }

  //clean the object list based on the timestamp
  void SimplePickPlace::cleanObjects()
  {
    ros::Time now = ros::Time::now() - ros::Duration(60);

    for (int i=0; i <blocks.size(); ++i)
    {
      if (blocks[i].timestamp < now)
        blocks.erase(blocks.begin()+i);
    }
  }

  void SimplePickPlace::updateObjects(const moveit_msgs::CollisionObject::ConstPtr& msg)
  {
    cleanObjects();

    try {
      if (msg->primitive_poses.size() > 0)
      {
        //check if exists
        int idx = -1;
        for (int i=0; i<blocks.size(); ++i)
          if (blocks[i].name == msg->id){
            idx = i;
            break;
          }

        if (idx == -1)
        {
         blocks.push_back(MetaBlock(msg->id, msg->header.stamp, msg->primitive_poses[0], shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
         msg_obj_poses.poses.push_back(blocks.back().start_pose);
        }
        else
          blocks[idx].updatePose(msg->primitive_poses[0]);
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }



  bool SimplePickPlace::startRoutine()
  {
    int block_id = 0;
    int actionState = 0; //nothing is done
    int actionDesired = 0; //reach

    while(ros::ok())
    {
      // Re-add all blocks
      /*for (std::size_t i = 0; i < blocks.size(); ++i)
        resetBlock(blocks[i]);*/

      if (block_id >= blocks.size())
      {
        block_id = 0;
        blocks.clear();
        msg_obj_poses.poses.clear();
        pub_obj_poses.publish(msg_obj_poses);
        break;
      }

      // Do for all blocks
      //for (std::size_t block_id = 0; block_id < blocks.size(); ++block_id)
      //{
        actionDesired = promptUserAction(blocks[block_id].name, blocks.size());
        ROS_INFO_STREAM_NAMED("pick_place:","Action chosen '" << actionDesired << "'"
                              << " object is " << block_id << " actionState is " << actionState);

        // Pick -------------------------------------------------------------------------------------
        if ((actionState == 0) && ((actionDesired == 1) || (actionDesired == 2)))
        {
          if(!action.pickAction(&blocks[block_id]))
          {
            /*if (saveStat_)
              poses_failed.push_back(blocks[block_id].start_pose);*/

            ROS_ERROR_STREAM_NAMED("pick_place","Pick failed.");
            resetBlock(blocks[block_id]);
            ++block_id;
            break;
          }
          else
          {
            /*if (saveStat_)
              poses_validated.push_back(blocks[block_id].start_pose);*/

            actionState += 2; //approach to object and grasp
            ROS_INFO_STREAM_NAMED("pick_place","Pick success ---------------------------");
            sleep(1.0);
          }
        } //end pick

        // Place -------------------------------------------------------------------------------------
        else if ((actionState == 2) && ((actionDesired == 3) || (actionDesired == 4)))
        {
          if(!action.placeAction(&blocks[block_id]))
          {
            ROS_ERROR_STREAM_NAMED("pick_place","Place failed.");
            ++block_id;
            break;
          }
          else
          {
            // Swap this block's start and end pose so that we can then move them back to position
            swapPoses(&blocks[block_id].start_pose, &blocks[block_id].goal_pose);

            ++actionState;
            ROS_INFO_STREAM_NAMED("pick_place","Place success ----------------------------");
            sleep(1.0);
          }
        } // end place

        // Return to the initial pose ---------------------------------------------------------------------
        if ((actionDesired == 5))
        {
          if(!action.returnAction(action.planning_group_name_))
          {
            ROS_ERROR_STREAM_NAMED("pick_place","Return failed.");
            break;
          }
          else
          {
            if (actionState == 3)
              swapPoses(&blocks[block_id].goal_pose, &blocks[block_id].start_pose);
            resetBlock(blocks[block_id]);

            actionState = 0;
            ROS_INFO_STREAM_NAMED("pick_place","Return success ----------------------------");
            sleep(1.0);
          }
        } // end place
        else if ((actionDesired == 6))
          break;

      //} // loop through blocks
    }

    return true;
  }

  void SimplePickPlace::resetBlock(MetaBlock block) //TODO
  {
    // Remove attached object
    visual_tools_->cleanupACO(block.name);

    // Remove collision object
    visual_tools_->cleanupCO(block.name);

    // Add the collision block
    publishCollisionMetaBlock(&block);
  }

  void SimplePickPlace::publishMetaBlock(MetaBlock *block, rviz_visual_tools::colors color, const bool goal)
  {
    geometry_msgs::Pose poseToPublish;
    if (!goal)
      poseToPublish = block->start_pose;
    else
      poseToPublish = block->goal_pose;

    if (block->shape.type == shape_msgs::SolidPrimitive::BOX)
      visual_tools_->publishBlock(poseToPublish, color, BLOCK_SIZE);
    else if (block->shape.type == shape_msgs::SolidPrimitive::CYLINDER)
      visual_tools_->publishCylinder(poseToPublish, color, BLOCK_SIZE*2, BLOCK_SIZE);
  }

  void SimplePickPlace::publishCollisionMetaBlock(MetaBlock *block)
  {
    if (block->shape.type == shape_msgs::SolidPrimitive::BOX)
      visual_tools_->publishCollisionBlock(block->start_pose, block->name, BLOCK_SIZE);
    else if (block->shape.type == shape_msgs::SolidPrimitive::CYLINDER)
      visual_tools_->publishCollisionCylinder(block->start_pose, block->name, BLOCK_SIZE, BLOCK_SIZE*3);
  }

  void SimplePickPlace::updateObjectsVirtual()
  {
    blocks.clear();
    //create objects at positions (hard coded)
    double block_x = 0.5;//0.47;
    double block_y = 0.2;//0.2;//0.4;
    double block_y_move = 0.06;

    blocks.push_back(MetaBlock("Block1", block_x, block_y, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
    //blocks.push_back(MetaBlock("Block2", block_x, block_y+0.08, shape_msgs::SolidPrimitive::BOX));

    block_y *= -1;
    block_y_move *= -1;
    blocks.push_back(MetaBlock("Block1", block_x, block_y, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));

    // The goal for each block is simply translating them on the y axis
    for (std::size_t i = 0; i < blocks.size(); ++i)
      blocks[i].goal_pose.position.y -= block_y_move;
  }

} //namespace

int main(int argc, char **argv)
{
  ros::init (argc, argv, "romeo_pick_place_adv");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }

  // Start the pick place node
  romeo_pick_place::SimplePickPlace server(verbose);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

/*std::vector<std::string> SimplePickPlace::getKnownObjectNames()
{
std::cout << " ---- planning_interface PlanningSceneInterface 181"<< std::endl;
  blocks.clear();
  try
  {
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    std::vector<std::string> result;
    request.components.components = request.components.WORLD_OBJECT_GEOMETRY;
    if (!planning_scene_service_.call(request, response))
    {
      ROS_WARN("Could not call planning scene service to get object names");
      return result;
    }
std::cout << " ---- planning_interface PlanningSceneInterface 194 response.scene.world.collision_objects.size() " << response.scene.world.collision_objects.size() << std::endl;
    for (std::size_t i = 0; i < response.scene.world.collision_objects.size() ; ++i)
    {

      std::cout << response.scene.world.collision_objects[i].primitive_poses.size()
                << " " << response.scene.world.collision_objects[i].mesh_poses.size() << std::endl;
      //blocks.push_back(MetaBlock(response.scene.world.collision_objects[i].id,
                                 //response.scene.world.collision_objects[i].mesh_poses,
                                 //shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
    }
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}*/


