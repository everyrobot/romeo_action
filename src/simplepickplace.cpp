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

  SimplePickPlace::SimplePickPlace(const bool verbose)
    :
      nh_("~"),
      nh_priv_(""),
      verbose_(verbose),
      saveStat_(true),
      actions_nbr(5),
      base_frame("base_link"),
      objproc(&nh_priv_)
  {
    // objects related initialization
    sub_obj_coll = nh_.subscribe<moveit_msgs::CollisionObject>("/collision_object", 10, &SimplePickPlace::getCollisionObjects, this);
    pub_obj_coll = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object",10);
    pub_obj_poses = nh_.advertise<geometry_msgs::PoseArray>("/obj_poses", 10);
    pub_obj_pose = nh_.advertise<geometry_msgs::PoseStamped>("/poses_reachable_current", 10);
    //planning_scene_service_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

    msg_obj_pose.header.frame_id = base_frame;//grasp_data_.base_link_;
    msg_obj_poses.header.frame_id = base_frame;//grasp_data_.base_link_;

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(base_frame));//grasp_data_.base_link_
    visual_tools_->setFloorToBaseHeight(-0.9);
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    cleanEnvironment(visual_tools_);
    // Let everything load
    ros::Duration(1.0).sleep();
    // Show grasp visualizations or not
    visual_tools_->setMuted(false);

    action_left = new Action(&nh_, visual_tools_, "left");
    action_right = new Action(&nh_, visual_tools_, "right");

    //move the robot to the initial position
    /*if (promptUserQuestion("Should I move the head to look down?"))
      posture.poseHeadDown();*/
    //if (promptUserQuestion("Should I move the left hand to the initial pose?"))
      posture.poseHandInit(action_left->end_eff, action_left->plan_group, action_left->arm);
    //sleep(2.0);
    //if (promptUserQuestion("Should I move the right hand to the initial pose?"))
      posture.poseHandInit(action_right->end_eff, action_right->plan_group, action_right->arm);
    //sleep(2.0);

    // Create the table
    createEnvironment(visual_tools_);

    //testReachablePoses();

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

    int count = 0;
    //for (float y=-0.3f; y<=-0.3f; y+=0.1f)
    for (float y=-0.2f; y<=0.2f; y+=0.1f)
      //for (float z=-0.2f; z<=-0.05f; z+=0.05f)
      for (float z=-0.15f; z<=-0.1f; z+=0.05f)
        //for (float x=0.3f; x<=0.5f; x+=0.05f)
        for (float x=0.35f; x<=0.5f; x+=0.05f)
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
            blocks_test.push_back(MetaBlock("BlockTest", x, y, z, 0,0,0,0, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
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
      blocks_test.push_back(MetaBlock("BlockTest", x, y, z, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
      msg_poses.poses.push_back(blocks_test.back().start_pose);
      ++count;
    }
    return msg_poses;
  }

  void SimplePickPlace::testReachablePoses(bool test_poses_rnd)
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
    testReachableSingleHand(action_left, &pub_obj_poses_left, blocks_test);
    testReachableSingleHand(action_right, &pub_obj_poses_right, blocks_test);
  }

  void SimplePickPlace::testReachableSingleHand(Action *action, ros::Publisher *pub_obj_poses, std::vector<MetaBlock> &blocks_test)
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

        if (action->pickAction(&(*block), ""))
        {
          std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv reachable pose: " << block->start_pose.position.x << " " << block->start_pose.position.y << " " << block->start_pose.position.z << std::endl;
          msg_poses_validated.poses.push_back(block->start_pose);
          pub_obj_poses->publish(msg_poses_validated);

          //return the hand
          posture.poseHandInit(action->end_eff, action->plan_group, action->arm);
          sleep(1.5);
          ++count_reachable;
        }
        // Remove attached object and Remove collision object
        visual_tools_->cleanupACO(block->name);
        visual_tools_->cleanupCO(block->name);
      }

    //return the hand
    posture.poseHandInit(action->end_eff, action->plan_group, action->arm);
    sleep(1.5);

    //print all reachable poses
    ROS_INFO_STREAM_NAMED("pick_place"," number of reachable objects = " << count_reachable << "/" << blocks_test.size());
    for (int i =0; i < msg_poses_validated.poses.size(); ++i)
      std::cout << msg_poses_validated.poses[i].position.x << " " << msg_poses_validated.poses[i].position.y << " " << msg_poses_validated.poses[i].position.z << std::endl;
  }

  bool SimplePickPlace::startRoutine()
  {
    int block_id = 0;
    int hand_id = 1; //actionDesired / actions_nbr;

    //publish a virtual object
    //updateObjectsVirtual();
    blocks.push_back(MetaBlock("Virtual1", 0.45, 0.3, -0.1, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
    //blocks.push_back(MetaBlock("Virtual1", 0.45, 0.2, -0.15, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
    msg_obj_pose.pose = blocks.back().start_pose;
    pub_obj_pose.publish(msg_obj_pose);

    //detect objects
    ros::Time start_time = ros::Time::now();
    while (nh_.ok() && (blocks.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(1.0)))
    {
      objproc.triggerObjectDetection();
    }

    // publish all objects as collision blocks
    if (blocks.size() > 0)
    {
      for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
        visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));
    }
    else
      block_id = -1;

    //testReachablePoses();

    while(ros::ok()) //main loop
    {
      //ROS_INFO_STREAM_NAMED("pick_place:", blocks.size() << " objects found" );
      int actionState = 0; //nothing is done
      int actionDesired = 0; //reach

      while(ros::ok()) //loop objects
      {
        ROS_INFO_STREAM_NAMED("pick_place:", block_id << " out of " << blocks.size() << " objects found" );
        std::cout << std::endl;
        if (block_id >= blocks.size()) //check if there are some objects
        {
          block_id = 0;
          if (block_id >= blocks.size()) //check if there are some objects
          {
            block_id = -1;
            removeObjects();

            objproc.triggerObjectDetection();
            // publish the collision block
            if (blocks.size() > 0)
            {
              block_id = 0;
              for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
                visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));
            }
            else
              block_id = -1;

            break;//continue;
          }
        }

        if ((actionState == 0) && (block_id != -1))
          resetBlock(&blocks[block_id]);
        msg_obj_pose.pose = blocks[block_id].start_pose;
        pub_obj_pose.publish(msg_obj_pose);

        actionDesired = promptUserAction(blocks[block_id].name, blocks.size());
        Action *action = action_left;
        if ((hand_id == 0) || (hand_id == 1))
        {
          if (hand_id == 0)
            action = action_right;
          else
            action = action_left;
          //actionDesired %= actions_nbr;
        }
        ROS_INFO_STREAM_NAMED("pick_place:","Action chosen '" << actionDesired << "'"
                              << " object=" << block_id << " actionState=" << actionState
                              << " actionDesired=" << actionDesired << " hand_id=" << hand_id);

        // Pick -----------------------------------------------------
        if ((actionState == 0) && (block_id != -1) && (actionDesired == 112)) //key 'p'
        {
          if(!action->pickAction(&blocks[block_id], SUPPORT_SURFACE3_NAME))
          {
            resetBlock(&blocks[block_id]);
            ++block_id;
          }
          else
          {
            actionState += 2; //approach to object and grasp
          }
        }
        // Place --------------------------------------------------------
        else if ((actionState == 2) && (block_id != -1) && ((actionDesired == 3) || (actionDesired == 4)))
        {
          if(action->placeAction(&blocks[block_id]))
          {
            // Swap this block's start and end pose so that we can then move them back to position
            swapPoses(&blocks[block_id].start_pose, &blocks[block_id].goal_pose);
            ++actionState;
          }
        }
        // Return the hand to the zero pose ------------------------------
        else if ((actionDesired == 122)) //key 'z'
        {
          // Remove attached object and Remove collision object
          visual_tools_->cleanupACO(blocks[block_id].name);
          visual_tools_->cleanupCO(blocks[block_id].name);

          if(posture.poseHandZero(action->end_eff, action->plan_group))
          {
            if (actionState == 3)
              swapPoses(&blocks[block_id].goal_pose, &blocks[block_id].start_pose);
            resetBlock(&blocks[block_id]);

            actionState = 0;
          }
        }
        //return the hand to the initial pose ------------------------------
        else if (actionDesired == 105) //key 'i'
        {
          // Remove attached object and Remove collision object
          visual_tools_->cleanupACO(blocks[block_id].name);
          visual_tools_->cleanupCO(blocks[block_id].name);

          if (posture.poseHandInit(action->end_eff, action->plan_group, action->arm))
          {
            if (actionState == 3)
              swapPoses(&blocks[block_id].goal_pose, &blocks[block_id].start_pose);
            resetBlock(&blocks[block_id]);

            actionState = 0;
          }
        }
        //return the hand to the pre-grasp pose ------------------------------
        else if (actionDesired == 103) //key 'g'
        {
          // Remove attached object and Remove collision object
          visual_tools_->cleanupACO(blocks[block_id].name);
          visual_tools_->cleanupCO(blocks[block_id].name);

          posture.poseHandPregrasp(action->end_eff, action->plan_group, action->arm);
        }
        //exit
        else if (actionDesired == 113) //key 'q'
          break;
        //create a virtual object for teh left hand
        else if (actionDesired == 108) //key 'l'
        {
          removeObjects();
          //updateObjectsVirtual();
          blocks.push_back(MetaBlock("Virtual1", 0.44, 0.05, -0.05, -1,0,0,0, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
          //blocks.push_back(MetaBlock("Virtual1", 0.45, 0.2, -0.15, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
          //blocks.push_back(MetaBlock("Virtual1", 0.3, -0.1, -0.05, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          // publish the collision block
          publishCollisionMetaBlock(&blocks.back());
        }
        else if (actionDesired == 114) //key 'r'
        {
          removeObjects();
          //updateObjectsVirtual();
          blocks.push_back(MetaBlock("Virtual1", 0.45, -0.2, -0.15, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
          //blocks.push_back(MetaBlock("Virtual1", 0.3, 0.1, -0.05, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          // publish the collision block
          publishCollisionMetaBlock(&blocks.back());
        }
        else if (actionDesired == 100) //key 'd'
        {
          removeObjects();
          objproc.triggerObjectDetection();
          // publish the collision block
          if (blocks.size() > 0)
          {
            block_id = 0;
            for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
              visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));
          }
          else
            block_id = -1;
        }
        else if (actionDesired == 110) //key 'n'
          ++block_id;
        else if (actionDesired == 116) //key 't'
          testReachablePoses();
        else if (actionDesired == 99) //key 'c'
        {
          cleanEnvironment(visual_tools_);
          removeObjects();
        }
        else if (actionDesired == 104) //key 'h'
        {
          if (hand_id == 0)
            ++hand_id;
          else
            hand_id = 0;
        }
        //moving the virtual objects
        else if (actionDesired == 50) //down
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.z -= 0.05;
            blocks[0].updatePose(pose);
            ROS_INFO_STREAM_NAMED("pick_place_adv", "new object pose: " << blocks[0].start_pose);
          }
        }
        else if (actionDesired == 52) //left
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.y -= 0.05;
            blocks[0].updatePose(pose);
            ROS_INFO_STREAM_NAMED("pick_place_adv", "new object pose: " << blocks[0].start_pose);
          }
        }
        else if (actionDesired == 56) //up
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.z += 0.05;
            blocks[0].updatePose(pose);
            ROS_INFO_STREAM_NAMED("pick_place_adv", "new object pose: " << blocks[0].start_pose);
          }
        }
        else if (actionDesired == 54) //right
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.y += 0.05;
            blocks[0].updatePose(pose);
            ROS_INFO_STREAM_NAMED("pick_place_adv", "new object pose: " << blocks[0].start_pose);
          }
        }
        else if (actionDesired == 51) //farther
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.x += 0.05;
            blocks[0].updatePose(pose);
            ROS_INFO_STREAM_NAMED("pick_place_adv", "new object pose: " << blocks[0].start_pose);
          }
        }
        else if (actionDesired == 57) //close
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.x -= 0.05;
            blocks[0].updatePose(pose);
            ROS_INFO_STREAM_NAMED("pick_place_adv", "new object pose: " << blocks[0].start_pose);
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
      }

      if (actionDesired == 113)
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



  //move to metablock
  moveit_msgs::CollisionObject SimplePickPlace::wrapToCollisionObject(MetaBlock *block)
  {
    moveit_msgs::CollisionObject msg_obj_collision;
    msg_obj_collision.header.stamp = ros::Time::now();
    msg_obj_collision.header.frame_id = base_frame;

    msg_obj_collision.id = block->name;
    msg_obj_collision.operation = moveit_msgs::CollisionObject::ADD;

    object_recognition_msgs::GetObjectInformation obj_info;
    obj_info.request.type = block->type;
    if (objproc.getMeshFromDB(obj_info))
    {
      msg_obj_collision.meshes.push_back(obj_info.response.information.ground_truth_mesh);
      msg_obj_collision.mesh_poses.push_back(block->start_pose);
    }
    else
    {
      shape_msgs::SolidPrimitive msg_cylinder_;
      msg_cylinder_.type = shape_msgs::SolidPrimitive::CYLINDER;
      msg_cylinder_.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
      msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = BLOCK_SIZE;
      msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = BLOCK_SIZE*3.0;

      msg_obj_collision.primitives.push_back(msg_cylinder_);
      msg_obj_collision.primitive_poses.push_back(block->start_pose);
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
      visual_tools_->publishCollisionBlock(block->start_pose, block->name, BLOCK_SIZE);
      //visual_tools_->publishCollisionBlock(block->start_pose, block->name, block->shape.BOX_X);
    else if (block->shape.type == shape_msgs::SolidPrimitive::CYLINDER)
      visual_tools_->publishCollisionCylinder(block->start_pose, block->name, BLOCK_SIZE, BLOCK_SIZE*3);
      //visual_tools_->publishCollisionCylinder(block->start_pose, block->name, block->shape.CYLINDER_RADIUS, block->shape.CYLINDER_HEIGHT);
  }

  void SimplePickPlace::getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg)
  {
    cleanObjects();

    try {
      if (msg->meshes.size() > 0)
      {
        //check if exists
        int idx = -1;
        for (int i=0; i<blocks.size(); ++i)
          if (blocks[i].name == msg->id){
            idx = i;
            break;
          }
        const geometry_msgs::Pose pose = msg->mesh_poses[0];
        /*if ((pose.position.x >= 0.3) && (pose.position.x <= 0.55)
            && (pose.position.y >= -0.45) && (pose.position.y <= 0.45)
            && (pose.position.z >= -0.24) && (pose.position.z <= -0.05))*/
        if ((idx == -1) || (idx >= blocks.size()) || (idx >= msg_obj_poses.poses.size())) //if not found, create a new one
        {
          blocks.push_back(MetaBlock(msg->id, msg->header.stamp, pose, msg->meshes.front(), msg->type));
          msg_obj_poses.poses.push_back(blocks.back().start_pose);
        }
        else if ((idx >= 0) && (idx < blocks.size()))
        {
          blocks[idx].updatePose(pose);
          msg_obj_poses.poses[idx] = blocks.back().start_pose;
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

  void SimplePickPlace::updateObjectsVirtual()
  {
    //create objects at positions (hard coded)
    double block_x = 0.3; //0.3; //0.3; //0.5;//0.47;
    double block_y = 0.1; //0.05; //0.05; //-0.355; //-0.2;//0.2;//0.2;//0.4;
    double block_z = -0.05; //-0.08; //-0.24; //-0.13;

    blocks.push_back(MetaBlock("Block1", block_x, block_y, block_z, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
    msg_obj_poses.poses.push_back(blocks.back().start_pose);

    /*blocks.push_back(MetaBlock("Block2", block_x, -1*block_y, block_z, shape_msgs::SolidPrimitive::CYLINDER, BLOCK_SIZE));
    msg_obj_poses.poses.push_back(blocks.back().start_pose);*/
  }

} //namespace

/*std::vector<std::string> SimplePickPlace::getKnownObjectNames()
{
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

      std::cout << response.scene.world.collision_objects[i].mesh_poses.size()
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


