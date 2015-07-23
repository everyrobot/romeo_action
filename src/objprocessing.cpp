#include "romeo_pick_place/objprocessing.hpp"

template<typename T>
void waitForAction(const T &action, const ros::NodeHandle &node_handle,
                                        const ros::Duration &wait_for_server, const std::string &name)
{
  ROS_DEBUG("Waiting for MoveGroup action server (%s)...", name.c_str());

  // in case ROS time is published, wait for the time data to arrive
  ros::Time start_time = ros::Time::now();
  while (start_time == ros::Time::now())
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  // wait for the server (and spin as needed)
  if (wait_for_server == ros::Duration(0, 0))
  {
    // wait forever until action server connects
    while (node_handle.ok() && !action->isServerConnected())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }
  else
  {
    // wait for a limited amount of non-simulated time
    ros::WallTime final_time = ros::WallTime::now() + ros::WallDuration(wait_for_server.toSec());
    while (node_handle.ok() && !action->isServerConnected() && final_time > ros::WallTime::now())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }

  if (!action->isServerConnected())
    throw std::runtime_error("Unable to connect to move_group action server within allotted time");
  else
    ROS_DEBUG("Connected to '%s'", name.c_str());
}

Objprocessing::Objprocessing(ros::NodeHandle *nh_):
  nh_(nh_),
  mesh_srv_name("get_object_info"),
  OBJECT_RECOGNITION_ACTION("/recognize_objects"),
  target_frame("base_link"),
  depth_frame_id("CameraDepth_frame")
{
  found_srv_obj_info = true;
  ros::Time start_time = ros::Time::now();
  while ((nh_->ok()) && (!ros::service::waitForService(mesh_srv_name, ros::Duration(2.0))))
  {
    ROS_INFO("Waiting for %s service to come up", mesh_srv_name.c_str());
    if (ros::Time::now() - start_time >= ros::Duration(5.0))
    {
      found_srv_obj_info = false;
      break;
    }
  }

  if (found_srv_obj_info)
  {
    get_model_mesh_srv_ = nh_->serviceClient<object_recognition_msgs::GetObjectInformation>
      (mesh_srv_name, false);
  }
}

bool Objprocessing::getMeshFromDB(object_recognition_msgs::GetObjectInformation &obj_info)
{
  if (!found_srv_obj_info)
    return false;

  if ( !get_model_mesh_srv_.call(obj_info) )
  {
    ROS_ERROR("The service get_object_info does not respond");
    return false;
  }
  return true;
}

void Objprocessing::triggerObjectDetection()
{
  if(!object_recognition_client_)
  {
    ROS_INFO_STREAM("Waiting for the Object recognition client");
    object_recognition_client_.reset(new actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>(OBJECT_RECOGNITION_ACTION, false));

    try
    {
      if (object_recognition_client_)
      {
        waitForAction(object_recognition_client_, *nh_, ros::Duration(2.0), OBJECT_RECOGNITION_ACTION);
        ROS_INFO_STREAM("Object recognition client is ready");
      }
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Object recognition action: %s", ex.what());
      return;
    }
  }

  if (object_recognition_client_)
  {
    object_recognition_msgs::ObjectRecognitionGoal goal;
    object_recognition_client_->sendGoal(goal);
    if (!object_recognition_client_->waitForResult())
    {
      ROS_INFO_STREAM("Object recognition client returned early");
    }
    if (object_recognition_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_WARN_STREAM("Fail: " << object_recognition_client_->getState().toString() << ": " << object_recognition_client_->getState().getText());
    }
  }
}
