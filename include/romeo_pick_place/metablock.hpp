#ifndef METABLOCK_H
#define METABLOCK_H

#include <geometry_msgs/Pose.h>
#include <shape_tools/solid_primitive_dims.h>

class MetaBlock
{
public:
  MetaBlock(const std::string name,
            const double start_x,
            const double start_y,
            const uint shapeType,
            const double size);
  MetaBlock(const std::string name,
            const ros::Time timestamp,
            const geometry_msgs::Pose start_pose,
            const uint shapeType,
            const double size);
  void updatePose(const geometry_msgs::Pose start_pose);
  void applyRndPose();

  std::string name;
  ros::Time timestamp;
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;
  shape_msgs::SolidPrimitive shape;
};


#endif // METABLOCK_H
