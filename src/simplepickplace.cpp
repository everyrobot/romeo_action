//publish messages with objects poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <romeo_pick_place/simplepickplace.hpp>
#include <romeo_pick_place/custom_environment5.hpp>
#include <romeo_pick_place/tools.hpp>

namespace romeo_pick_place
{
  void swapPoses(geometry_msgs::Pose *pose1, geometry_msgs::Pose *pose2){
    geometry_msgs::Pose temp = *pose1;
    pose1 = pose2;
    *pose2 = temp;
  }

  SimplePickPlace::SimplePickPlace(const std::string robot_name, const bool verbose)
    :
      nh_("~"),
      nh_priv_(""),
      robot_name_(robot_name),
      verbose_(verbose),
      saveStat_(true),
      base_frame("odom"), //base_link
      block_size(0.03), //0.03 //0.032
      block_size_r(0.15), //0.11 //3.67), //2.22
      env_shown_(false),
      x_min(0.35),
      x_max(0.5),
      z_min(-0.17),
      z_max(-0.05),
      pose_default(),
      pose_default_r(),
      objproc(&nh_priv_)
  {
    pose_default.orientation.x = -1;
    if (robot_name_ == "nao")
    {
      block_size = 0.01;
      pose_default.position.x = 0.2;
      pose_default.position.y = 0.1;
      pose_default.position.z = 0.0;
    }
    else if (robot_name == "pepper")
    {
      block_size = 0.02;
      pose_default.position.x = 0.25;
      pose_default.position.y = 0.2;
      pose_default.position.z = -0.1;
    }
    else
    {
      pose_default.position.x = 0.44;
      pose_default.position.y = 0.15;
      pose_default.position.z = -0.1;
    }

    pose_default_r = pose_default;
    pose_default_r.position.y *= -1;

    // objects related initialization
    sub_obj_coll = nh_.subscribe<moveit_msgs::CollisionObject>("/collision_object", 10, &SimplePickPlace::getCollisionObjects, this);
    //pub_obj_coll = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object",10);
    pub_obj_poses = nh_.advertise<geometry_msgs::PoseArray>("/obj_poses", 10);
    pub_obj_pose = nh_.advertise<geometry_msgs::PoseStamped>("/pose_current", 10);

    //planning_scene_service_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

    msg_obj_pose.header.frame_id = base_frame;//grasp_data_.base_link_;
    msg_obj_poses.header.frame_id = base_frame;//grasp_data_.base_link_;

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(base_frame));//grasp_data_.base_link_
    visual_tools_->setFloorToBaseHeight(-0.9);
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->setMuted(false);
    cleanEnvironment(visual_tools_);
    env_shown_ = false;
    ros::Duration(1.0).sleep();

    action_left = new Action(&nh_, visual_tools_, "left", robot_name_);
    action_right = new Action(&nh_, visual_tools_, "right", robot_name_);

    //move the robot to the initial position
    //if (promptUserQuestion("Should I move the head to look down?"))
      //action_left->poseHeadDown();
    //if (promptUserQuestion(("Should I move the "+action_left->end_eff+" to the initial pose?").c_str()))
      //action_left->poseHandInit();
    std::vector<double> pose_arm;
    pose_arm.resize(6, 0.0);
    if (robot_name_ == "romeo")
    {
      pose_arm[0] = 1.74;
      pose_arm[1] = 0.7;
      pose_arm[2] = -2.08;
      pose_arm[3] = -1.15;
      pose_arm[4] = -0.43;
      pose_arm[5] = 0.17;
    }
    action_left->poseHand(&pose_arm);

    //if (promptUserQuestion(("Should I move the "+action_right->plan_group+" to the initial pose?").c_str()))
      //action_right->poseHandInit();
    if (robot_name_ == "romeo")
    {
      pose_arm[1] *= -1;
      pose_arm[2] *= -1;
      pose_arm[3] *= -1;
    }
    action_right->poseHand(&pose_arm);

    // Create the table
    createEnvironment(visual_tools_);
    env_shown_ = true;

    startRoutine();
  }

  geometry_msgs::PoseArray SimplePickPlace::generatePosesGrid(std::vector<MetaBlock> &blocks_test)
  {
    blocks_test.clear();
    geometry_msgs::PoseArray msg_poses;
    msg_poses.header.frame_id = base_frame;//grasp_data_.base_link_;

    ROS_INFO_STREAM_NAMED("pick_place"," Looking for objects");

    //detect objects to get any mesh
    ros::Time start_time = ros::Time::now();
    while (nh_.ok() && (blocks.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(1.0)))
    {
      objproc.triggerObjectDetection();
    }

    ROS_INFO_STREAM_NAMED("pick_place"," Generating goals in the target space");

    double step = 0.04;
    if (robot_name_ == "nao")
    {
      step = 0.03;
      x_min = 0.1; //0.3;
      x_max = 0.21;
      z_min = -0.07;
      z_max = +0.05;
    }
    else if (robot_name_ == "pepper")
    {
      step = 0.04;
      x_min = 0.2; //0.3;
      x_max = 0.4;
      z_min = -0.13;
      z_max = 0.0;
    }

    int count = 0;
    //for (double y=-0.3; y<=-0.3; y+=0.1)
    for (double y=-0.2; y<=0.2; y+=step*2.0)
      for (double z=z_min; z<=z_max; z+=step)
        for (double x=x_min; x<=x_max; x+=step)
        {
          if (blocks.size() > 0)
          {
            MetaBlock b = blocks.front();
            b.start_pose.position.x = x;
            b.start_pose.position.y = y;
            b.start_pose.position.z = z;
            b.start_pose.orientation.x = -1.0; //-1.0; //
            b.start_pose.orientation.y = 0.0;
            b.start_pose.orientation.z = 0.0;
            b.start_pose.orientation.w = 0.0;
            blocks_test.push_back(b);
          }
          else
          {
            blocks_test.push_back(MetaBlock("BlockTest", x, y, z, 0,0,0,0, shape_msgs::SolidPrimitive::CYLINDER, block_size, block_size_r));
          }

          msg_poses.poses.push_back(blocks_test.back().start_pose);
          std::cout << x << " " << y << " " << z << std::endl;
          ++count;
        }

    ROS_INFO_STREAM_NAMED("pick_place","Total number of generated poses=" << count);
    return msg_poses;
  }

  geometry_msgs::PoseArray SimplePickPlace::generatePosesRnd(const int poses_nbr, std::vector<MetaBlock> &blocks_test)
  {
    blocks_test.clear();
    geometry_msgs::PoseArray msg_poses;
    msg_poses.header.frame_id = "base_link";//grasp_data_.base_link_;

    int count = 0;
    while (count < poses_nbr){
      float x = 0.35f + float(rand() % 150)/1000.0f; //[0.35;0.50]
      float y = float(rand() % 90)/100.0f - 0.45; //[-0.45;0.45]
      float z = -0.23f + (float(rand() % 230)/1000.0f); //[-0.23;0.00]
      blocks_test.push_back(MetaBlock("BlockTest", x, y, z, -1,0,0,0, shape_msgs::SolidPrimitive::CYLINDER, block_size, block_size_r));
      msg_poses.poses.push_back(blocks_test.back().start_pose);
      ++count;
    }
    return msg_poses;
  }

  void SimplePickPlace::testReachablePoses(const bool pickVsReach, bool test_poses_rnd)
  {
    geometry_msgs::PoseArray msg_poses_test;
    if (test_poses_rnd)
      msg_poses_test = generatePosesRnd(200, blocks_test);
    else
      msg_poses_test = generatePosesGrid(blocks_test);

    //visualize all generated samples of the goal space
    pub_obj_poses.publish(msg_poses_test);
    sleep(0.05);

    ros::Publisher pub_obj_poses_left = nh_.advertise<geometry_msgs::PoseArray>("/poses_reachable_left", 100);
    ros::Publisher pub_obj_poses_right = nh_.advertise<geometry_msgs::PoseArray>("/poses_reachable_right", 100);
    testReachableSingleHand(action_left, &pub_obj_poses_left, blocks_test, pickVsReach);
    testReachableSingleHand(action_right, &pub_obj_poses_right, blocks_test, pickVsReach);
  }

  void SimplePickPlace::testReachableSingleHand(Action *action, ros::Publisher *pub_obj_poses, std::vector<MetaBlock> &blocks_test, const bool pickVsReach)
  {
    int count_reachable = 0;
    geometry_msgs::PoseArray msg_poses_validated;
    msg_poses_validated.header.frame_id = "base_link";//grasp_data_.base_link_;

      for (std::vector<MetaBlock>::iterator block=blocks_test.begin(); block != blocks_test.end(); ++block)
        if ((action->arm == "left") && (block->start_pose.position.y >= 0.0f) ||
            (action->arm == "right") && (block->start_pose.position.y <= 0.0f))
      {
        // Publish the collision object
        //publishCollisionMetaBlock(&(*block));
        visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));

        msg_obj_pose.pose = block->start_pose;
        pub_obj_pose.publish(msg_obj_pose);

        bool success;
        if (pickVsReach)
          success = action->pickAction(&(*block), "");
        else
        {
          visual_tools_->cleanupCO(block->name);
          success = action->reachPregrasp(block->start_pose, "");
          resetBlock(&(*block));
        }
        if (success)
        {
          std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv reachable pose: " << block->start_pose.position.x << " " << block->start_pose.position.y << " " << block->start_pose.position.z << std::endl;
          msg_poses_validated.poses.push_back(block->start_pose);
          pub_obj_poses->publish(msg_poses_validated);

          //return the hand
          action->poseHandInit();
          sleep(1.5);
          ++count_reachable;
        }
        // Remove attached object and Remove collision object
        visual_tools_->cleanupACO(block->name);
        visual_tools_->cleanupCO(block->name);
      }

    //return the hand
    action->poseHandInit();
    sleep(1.5);

    //print all reachable poses
    ROS_INFO_STREAM_NAMED("pick_place"," number of reachable objects = " << count_reachable << "/" << blocks_test.size());
    for (int i =0; i < msg_poses_validated.poses.size(); ++i)
      std::cout << msg_poses_validated.poses[i].position.x << " " << msg_poses_validated.poses[i].position.y << " " << msg_poses_validated.poses[i].position.z << std::endl;
  }

  bool SimplePickPlace::startRoutine()
  {
    int block_id = -1;
    int hand_id = 1; //0: right, 1: left

    //publish a virtual object
    //updateObjectsVirtual();
    block_id = 0;
    blocks.push_back(MetaBlock("Virtual1", ros::Time::now(), pose_default, shape_msgs::SolidPrimitive::CYLINDER, block_size));
    //blocks.push_back(MetaBlock("Virtual1", 0.44, 0.15, -0.1, -1,0,0,0, shape_msgs::SolidPrimitive::CYLINDER, block_size));
    msg_obj_pose.pose = blocks.back().start_pose;
    pub_obj_pose.publish(msg_obj_pose);

    //detect objects
    /*ros::Time start_time = ros::Time::now();
    while (nh_.ok() && (blocks.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(1.0)))
    {
      objproc.triggerObjectDetection();
      ROS_INFO_STREAM("Trying to detect objects since " << ros::Time::now() - start_time);
    }
    // publish all objects as collision blocks
    if (blocks.size() > 0)
    {
      block_id = 0;
      for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
        visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));
    }
    else
      block_id = -1;*/

    while(ros::ok()) //main loop
    {
      //ROS_INFO_STREAM_NAMED("pick_place:", blocks.size() << " objects found" );
      //int actionState = 0; //nothing is done
      int actionDesired = 0; //reach
      std::string actionName = "";

      while(ros::ok()) //loop objects
      {
        ROS_INFO_STREAM_NAMED("pick_place:", block_id << " out of " << blocks.size() << " objects found" );
        if (block_id >= blocks.size()) //check if there are some objects
        {
          block_id = -1;
          removeObjects();
          objproc.triggerObjectDetection();
          // publish all objects as collision blocks
          if (blocks.size() > 0)
          {
            block_id = 0;
            for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
              visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));
          }
        }

        /*if ((block_id != -1)) //(actionState == 0) &&
          resetBlock(&blocks[block_id]);*/
        if (block_id != -1)
        {
          msg_obj_pose.pose = blocks[block_id].start_pose;
          pub_obj_pose.publish(msg_obj_pose);
        }

        std::string temp = "";
        if (blocks.size() > 0)
          temp = blocks[block_id].name;
        ROS_INFO_STREAM_NAMED("pick_place","What do you want me to do with the object "
                              << temp << " out of " << blocks.size() << " detected objects? ");
        actionName = promptUserQuestionString();
        //actionDesired = promptUserAction();

        Action *action = action_left;
        if ((hand_id == 0) || (hand_id == 1))
        {
          if (hand_id == 0)
            action = action_right;
          else
            action = action_left;
        }
        ROS_INFO_STREAM_NAMED("pick_place:","Action chosen '" << actionDesired << " " << actionName
                              << "' object=" << block_id
                              //<< " actionState=" << actionState
                              << " hand_id=" << hand_id);

        // Pick -----------------------------------------------------
        if ((block_id != -1) && ((actionDesired == 103) || (actionName == "g"))) //key 'g' //pick the object with a default function //(actionState == 0) &&
        {
          if(action->pickAction(&blocks[block_id], SUPPORT_SURFACE3_NAME))
          {
            //actionState += 2; //approach to object and grasp
          }
        }
        // Place --------------------------------------------------------
        else if ((block_id != -1) && ((actionDesired == 112) || (actionName == "p")))  //key 'p' //place the object with a default function //(actionState == 2) &&
        {
          if(action->placeAction(&blocks[block_id]))
          {
            // Swap this block's start and end pose so that we can then move them back to position
            swapPoses(&blocks[block_id].start_pose, &blocks[block_id].goal_pose);
            resetBlock(&blocks[block_id]);
            //++actionState = 0;
          }
        }
        // Return the hand to the zero pose ------------------------------
        else if ((actionDesired == 122) || (actionName == "z")) //key 'z' //move to zero pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          action->poseHandZero();
          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //return the hand to the initial pose ------------------------------
        else if ((actionDesired == 105) || (actionName == "i")) //key 'i' //move to init pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          action->poseHandInit();
          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //return the hand to the initial pose ------------------------------
        else if ((actionName == "i1")) //key 'i1' //move to init pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          std::vector<double> pose_arm_left;
          pose_arm_left.resize(6, 0.0);
          if (robot_name_ == "romeo")
          {
            pose_arm_left[0] = 1.0799225568771362;
            pose_arm_left[1] = 0.6565437912940979;
            pose_arm_left[2] = -0.8390874862670898;
            pose_arm_left[3] = -0.607456386089325;
            pose_arm_left[4] = -0.6672816872596741;
            pose_arm_left[5] = 0.02454369328916073;
          }

          action->poseHand(&pose_arm_left);
          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //return the hand to the initial pose ------------------------------
        else if ((actionName == "i2")) //key 'i2' //move to init pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          std::vector<double> pose_arm;
          pose_arm.resize(6, 0.0);
          if (robot_name_ == "romeo")
          {
            pose_arm[0] = 1.872990608215332;
            pose_arm[1] = 0.5553010702133179;
            pose_arm[2] = -1.9895731210708618;
            pose_arm[3] = -1.052310824394226;
            pose_arm[4] = -0.6703495979309082;
            pose_arm[5] = 0.02147573232650757;
          }

          action->poseHand(&pose_arm);
          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //return the hand to the initial pose ------------------------------
        else if ((actionName == "i3")) //key 'i3' //move to init pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          std::vector<double> pose_arm;
          pose_arm.resize(6, 0.0);
          if (robot_name_ == "romeo")
          {
            pose_arm[0] = 1.74;
            pose_arm[1] = 0.75;
            pose_arm[2] = -2.08;
            pose_arm[3] = -1.15;
            pose_arm[4] = -0.43;
            pose_arm[5] = 0.17;

            if (action->arm == "right")
            {
              pose_arm[1] *= -1;
              pose_arm[2] *= -1;
              pose_arm[3] *= -1;
            }
          }

          action->poseHand(&pose_arm);
          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //exit
        else if ((actionDesired == 113) || (actionName == "q")) //key 'q' //quit the application
          break;
        //create a virtual object for teh left hand
        else if ((actionDesired == 108) || (actionName == "l")) //key 'l' //clean and publish virtual for the left arm
        {
          removeObjects();
          //updateObjectsVirtual();
          blocks.push_back(MetaBlock("Virtual1", ros::Time::now(), pose_default, shape_msgs::SolidPrimitive::CYLINDER, block_size));
          //blocks.push_back(MetaBlock("Virtual1", 0.44, 0.15, -0.1, -1,0,0,0, shape_msgs::SolidPrimitive::CYLINDER, block_size));

          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          publishCollisionMetaBlock(&blocks.back());
        }
        else if ((actionName == "l2")) //key 'l2' //clean and publish virtual for the left arm
        {
          removeObjects();
          //updateObjectsVirtual();
          geometry_msgs::Pose pose_default_temp(pose_default);
          pose_default_temp.position.z -= 0.1;
          blocks.push_back(MetaBlock("Virtual1", ros::Time::now(), pose_default_temp, shape_msgs::SolidPrimitive::CYLINDER, block_size));

          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          publishCollisionMetaBlock(&blocks.back());
        }

        else if ((actionDesired == 114) || (actionName == "r")) //key 'r' //clean and publish virtual for the right arm
        {
          removeObjects();
          //updateObjectsVirtual();
          //side grasp
          blocks.push_back(MetaBlock("Virtual1", ros::Time(), pose_default_r, shape_msgs::SolidPrimitive::CYLINDER, block_size));
          //top grasp
          //blocks.push_back(MetaBlock("Virtual1", 0.45, -0.25, -0.1, -1,0,0,0, shape_msgs::SolidPrimitive::CYLINDER, block_size));

          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          publishCollisionMetaBlock(&blocks.back());
        }
        else if ((actionDesired == 100) || (actionName == "d")) //key 'd' //detect objects
        {
          block_id = -1;
          removeObjects();

          objproc.triggerObjectDetection();
          // publish all objects as collision blocks
          if (blocks.size() > 0)
          {
            block_id = 0;
            for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
            {
              visual_tools_->cleanupCO(block->name);

              //visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));
              moveit_msgs::CollisionObject msg = wrapToCollisionObject(&(*block));
              //if (!msg.meshes.empty())
               //{
              if (!msg.primitive_poses.empty())
                  ROS_INFO_STREAM("---- primitive_poses[0]" << msg.primitive_poses[0]);
                visual_tools_->processCollisionObjectMsg(msg);
              /*}
              else
              {
                ROS_INFO_STREAM("----- " << block->start_pose);
                publishCollisionMetaBlock(&(*block));
              }*/
            }
          }
          else
            block_id = -1;
        }
        //dd continuous object detection
        else if (actionName == "dd") //key 'dd' //detect objects
        {
          block_id = -1;
          removeObjects();

          ros::Time start_time = ros::Time::now();
          while ((blocks.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(15.0)))
          {
ROS_INFO_STREAM("**** object detection is running, objects detected  " << blocks.size() << " " << ros::Time::now());
            objproc.triggerObjectDetection();
          }

          // publish all objects as collision blocks
          if (blocks.size() > 0)
          {
            block_id = 0;
            for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
            {
              //visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));
              moveit_msgs::CollisionObject msg = wrapToCollisionObject(&(*block));
              if (!msg.primitive_poses.empty())
                  ROS_INFO_STREAM("---- primitive_poses[0]" << msg.primitive_poses[0]);
                visual_tools_->processCollisionObjectMsg(msg);
            }
          }
          else
            block_id = -1;
        }
        //------------ reaching generated poses by moveit simple grasps
        else if ((block_id != -1) && ((actionDesired == 97) || (actionName == "a"))) //key 'a' //plan movement
        {
          visual_tools_->cleanupCO(blocks[block_id].name);
          action->graspPlan(&blocks[block_id], SUPPORT_SURFACE3_NAME);
          resetBlock(&blocks[block_id]);
        }
        else if ((block_id != -1) && ((actionDesired == 102) || (actionName == "f"))) //key 'f' //plan all movement
        {
         visual_tools_->cleanupCO(blocks[block_id].name);
          action->graspPlanAllPossible(&blocks[block_id], SUPPORT_SURFACE3_NAME);
          resetBlock(&blocks[block_id]);
        }
        //------------ reaching default poses
        else if ((block_id != -1) && ((actionDesired == 117) || (actionName == "u"))) //key 'u' //reach and grasp
        {
          visual_tools_->cleanupCO(blocks[block_id].name);
          action->reachGrasp(blocks[block_id].start_pose, SUPPORT_SURFACE3_NAME);
          resetBlock(&blocks[block_id]);
        }
        else if ((block_id != -1) && ((actionDesired == 120) || (actionName == "x"))) //key 'x' //reach the pregrasp pose
        {
          visual_tools_->cleanupCO(blocks[block_id].name);
          action->reachPregrasp(blocks[block_id].start_pose, SUPPORT_SURFACE3_NAME);
          resetBlock(&blocks[block_id]);
        }
        else if ((block_id != -1) && ((actionDesired == 121) || (actionName == "y"))) //key 'y' //reach from top
        {
          visual_tools_->cleanupCO(blocks[block_id].name);
          geometry_msgs::Pose pose = blocks[block_id].start_pose;
          pose.orientation.x = 0;
          pose.position.z += blocks[block_id].size*1.5; //0.12;
          action->reachAction(pose, SUPPORT_SURFACE3_NAME);
          resetBlock(&blocks[block_id]);
        }
        else if ((block_id != -1) && ((actionDesired == 121) || (actionName == "b"))) //key 'b'
        {
          action->pickDefault(&blocks[block_id]);
        }
        else if ((block_id != -1) && ((actionDesired == 119) || (actionName == "w"))) //key 'w' //reach the init pose
        {
          visual_tools_->cleanupCO(blocks[block_id].name);
          action->reachInitPose();
          resetBlock(&blocks[block_id]);
        }
        //--------------
        else if ((block_id != -1) && ((actionDesired == 101) || (actionName == "e"))) //key 'e' //execute tha plan
        {
          action->executeAction();
        }
        else if ((actionDesired == 118) || (actionName == "v")) //'v' //print the current pose
        {
          action->getPose();
        }
        else if ((actionDesired == 111) || (actionName == "o")) //key 'o' //open hand
        {
          action->poseHandOpen();
        }
        else if ((actionDesired == 99) || (actionName == "c")) //key 'c' //close hand
        {
          action->poseHandOpen();
        }
        else if ((actionDesired == 110) || (actionName == "n")) //key 'n' //process the next object
          ++block_id;
        else if ((actionDesired == 116) || (actionName == "test_pick")) //key 't' //test the goal space for picking
          testReachablePoses(true);
        else if ((actionDesired == 116) || (actionName == "test_reach")) //key 't' //test the goal space for reaching
          testReachablePoses(false);
        else if ((actionDesired == 116) || (actionName == "test_ik")) //key 't' // Filter grasps for reaching by IK
          action->filterGrasps(&blocks[block_id]);
        else if ((actionDesired == 115) || (actionName == "s")) //key 's' //clean the scene
        {
          if (env_shown_)
          {
            cleanEnvironment(visual_tools_);
            env_shown_ = false;
            removeObjects();
          }
          else
          {
            createEnvironment(visual_tools_);
            env_shown_ = true;
          }
        }
        else if ((actionDesired == 104) || (actionName == "h")) //key 'h'
        {
          if (hand_id == 0)
            ++hand_id;
          else
            hand_id = 0;
        }
        //moving the virtual objects
        else if ((actionDesired == 50) || (actionName == "down")) //down
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[block_id].start_pose;
            pose.position.z -= 0.05;
            blocks[block_id].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 52) || (actionName == "left")) //left
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[block_id].start_pose;
            pose.position.y -= 0.05;
            blocks[block_id].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 56) || (actionName == "up")) //up
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[block_id].start_pose;
            pose.position.z += 0.05;
            blocks[block_id].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 54) || (actionName == "right")) //right
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[block_id].start_pose;
            pose.position.y += 0.05;
            blocks[block_id].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 51) || (actionName == "further")) //further
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.x += 0.05;
            blocks[0].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 57) || (actionName == "closer")) //closer
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.x -= 0.05;
            blocks[0].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if (actionDesired == 55) //num-1
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.orientation.x = -1;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 0;
            blocks[0].updatePose(pose);
            ROS_INFO_STREAM_NAMED("pick_place_adv", "new object pose: " << blocks[0].start_pose);
          }
        }
        else if (actionDesired == 49) //num-7
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 0;
            blocks[0].updatePose(pose);
            ROS_INFO_STREAM_NAMED("pick_place_adv", "new object pose: " << blocks[0].start_pose);
          }
        }
        else if ((actionDesired == 106) || (actionName == "j")) //key 'j', set the tolerance
        {
          action->setTolerance(promptUserValue("Set the value: "));
        }
        else if ((actionDesired == 109) || (actionName == "m")) //key 'm', move the head down
          action->poseHeadDown();
        else if ((actionDesired == 107) || (actionName == "k")) //key 'k', move the head to zero
          action->poseHeadZero();
      }

      if ((actionDesired == 113) || (actionName == "q"))
        break;
    }
    return true;
  }

  //clean the object list based on the timestamp
  void SimplePickPlace::cleanObjects()
  {
    ros::Time now = ros::Time::now() - ros::Duration(60);

    for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
    {
      if (block->timestamp < now)
      {
        // Remove attached object
        visual_tools_->cleanupACO(block->name);
        // Remove collision object
        visual_tools_->cleanupCO(block->name);

        //blocks.erase(block);
      }
    }
  }

  moveit_msgs::CollisionObject SimplePickPlace::wrapToCollisionObject(MetaBlock *block)
  {
    moveit_msgs::CollisionObject msg_obj_collision;
    msg_obj_collision.header.stamp = ros::Time::now();
    msg_obj_collision.header.frame_id = base_frame;

    msg_obj_collision.id = block->name;
    msg_obj_collision.operation = moveit_msgs::CollisionObject::ADD;

    object_recognition_msgs::GetObjectInformation obj_info;
    obj_info.request.type = block->type;

    /*if (objproc.getMeshFromDB(obj_info))
    {
      msg_obj_collision.meshes.push_back(obj_info.response.information.ground_truth_mesh);
      msg_obj_collision.mesh_poses.push_back(block->start_pose);
ROS_INFO_STREAM("-- mesh found: msg_obj_collision.meshes.size()=" << msg_obj_collision.meshes.size());
    }
    else*/
    {
      shape_msgs::SolidPrimitive msg_cylinder_;
      msg_cylinder_.type = shape_msgs::SolidPrimitive::CYLINDER;
      msg_cylinder_.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
      msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = block_size;
      msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = block_size_r; //block_size*

      msg_obj_collision.primitives.push_back(msg_cylinder_);

      msg_obj_collision.primitive_poses.push_back(block->start_pose);
//ROS_INFO_STREAM("-- mesh not found: " << block->start_pose);
    }

    msg_obj_collision.type = block->type;
    return msg_obj_collision;
  }

  void SimplePickPlace::resetBlock(MetaBlock *block)
  {
    // Remove attached object
    visual_tools_->cleanupACO(block->name);

    // Remove collision object
    visual_tools_->cleanupCO(block->name);

    // Add the collision block
    visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(block));
    //publishCollisionMetaBlock(block);
  }

  void SimplePickPlace::publishCollisionMetaBlock(MetaBlock *block)
  {
    if (block->shape.type == shape_msgs::SolidPrimitive::BOX)
      visual_tools_->publishCollisionBlock(block->start_pose, block->name, block_size);
      //visual_tools_->publishCollisionBlock(block->start_pose, block->name, block->shape.BOX_X);
    else if (block->shape.type == shape_msgs::SolidPrimitive::CYLINDER)
      visual_tools_->publishCollisionCylinder(block->start_pose, block->name, block_size, block_size_r); //block_size*
      //visual_tools_->publishCollisionCylinder(block->start_pose, block->name, block->shape.CYLINDER_RADIUS, block->shape.CYLINDER_HEIGHT);
  }

  void SimplePickPlace::getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg)
  {
    cleanObjects();

    try {
      if (msg->meshes.size() > 0)
      {
        geometry_msgs::Pose pose = msg->mesh_poses[0];
        if ((pose.position.x < x_max) && (pose.position.x > x_min)
            && (pose.position.z < z_max) && (pose.position.z > z_min))
        {
          //check if exists
          int idx = -1;
          for (int i=0; i<blocks.size(); ++i)
            if (blocks[i].name == msg->id){
              idx = i;
              break;
            }

          //pose.position.z += 0.0576;
          if ((idx == -1) || (idx >= blocks.size()) || (idx >= msg_obj_poses.poses.size())) //if not found, create a new one
          {
            blocks.push_back(MetaBlock(msg->id, msg->header.stamp, pose, msg->meshes.front(), msg->type));
            msg_obj_poses.poses.push_back(pose);//blocks.back().start_pose);
          }
          else if ((idx >= 0) && (idx < blocks.size()))
          {
            blocks[idx].updatePose(pose);
            msg_obj_poses.poses[idx] = pose; //blocks.back().start_pose; //
          }
        }
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  void SimplePickPlace::removeObjects()
  {
    //ROS_INFO_STREAM_NAMED("pick_place:","cleaning blocks");
    //remove from the scene
    for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
    {
      // Remove attached object
      visual_tools_->cleanupACO(block->name);
      // Remove collision object
      visual_tools_->cleanupCO(block->name);
    }

    //remove from the memory
    blocks.clear();
    msg_obj_poses.poses.clear();
    pub_obj_poses.publish(msg_obj_poses);
  }

} //namespace


