#include <romeo_pick_place/metablock.hpp>
#include <Eigen/Eigen>

MetaBlock::MetaBlock(const std::string name,
          const double start_x,
          const double start_y,
          const uint shapeType,
          const double size)
{
  this->name = name;

  //set position
  start_pose.position.x = start_x;
  start_pose.position.y = start_y;
  start_pose.position.z = -0.13;//-0.11; //getTableHeight(-0.9); //from -0.11 to -0.1

  //set orientation
  double angle = -M_PI; //M_PI / 1.5;
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitX()));
  start_pose.orientation.x = quat.x();
  start_pose.orientation.y = quat.y();
  start_pose.orientation.z = quat.z();
  start_pose.orientation.w = quat.w();
  //cube
  /*start_pose.orientation.x = -0.707;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = -0.0;
  start_pose.orientation.w = -0.707;*/
  //cyllinder
  /*start_pose.orientation.x = -1.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.0;*/

  goal_pose = start_pose;
  goal_pose.position.y -= 0.06;

  //setshape
  if (shapeType == shape_msgs::SolidPrimitive::CYLINDER)
  {
    shape_msgs::SolidPrimitive shapeCylinder;
    shapeCylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
    shapeCylinder.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    shapeCylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = size*3;
    shapeCylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = size;
    this->shape = shapeCylinder;
  }
  else
  {
    shape_msgs::SolidPrimitive shapeBox;
    shapeBox.type = shape_msgs::SolidPrimitive::BOX;
    shapeBox.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_X] = size;
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size;
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = size;
    this->shape = shapeBox;
  }
}

MetaBlock::MetaBlock(const std::string name,
          const ros::Time timestamp,
          const geometry_msgs::Pose start_pose,
          const uint shapeType,
          const double size)
{
  this->name = name;
  this->timestamp = timestamp;
  this->start_pose = start_pose;
  this->goal_pose = start_pose;
  this->goal_pose.position.y = 0.14;
  //std::cout << "MetaBlock:: detected " << this->start_pose.position << std::endl;

  //setshape
  if (shapeType == shape_msgs::SolidPrimitive::CYLINDER)
  {
    shape_msgs::SolidPrimitive shapeCylinder;
    shapeCylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
    shapeCylinder.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    shapeCylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = size*3;
    shapeCylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = size;
    this->shape = shapeCylinder;
  }
  else
  {
    shape_msgs::SolidPrimitive shapeBox;
    shapeBox.type = shape_msgs::SolidPrimitive::BOX;
    shapeBox.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_X] = size;
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size;
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = size;
    this->shape = shapeBox;
  }
}

void MetaBlock::updatePose(const geometry_msgs::Pose start_pose)
{
  this->start_pose = start_pose;
  this->goal_pose = start_pose;
  this->goal_pose.position.y -= 0.06;
  //std::cout << "MetaBlock:: detected " << this->start_pose.position << std::endl;
}
