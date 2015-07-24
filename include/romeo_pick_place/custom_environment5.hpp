
#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasps

#ifndef CUSTOM_ENVIRONMENT_
#define CUSTOM_ENVIRONMENT_

namespace romeo_pick_place
{

// table dimensions
std::string SUPPORT_SURFACE3_NAME = ""; //"table";
static const double TABLE_HEIGHT = 0.845 -0.1; //0.86; //0.81; //0.64; //0.71;//0.76;
static const double TABLE_WIDTH  = 0.3 + 2* 0.15; //0.47; //0.44; //0.86;
static const double TABLE_DEPTH  = 0.35; //0.04 + 0.4;//0.36; //0.86;//0.86;
static const double TABLE_X = 0.45; //0.4826 + TABLE_DEPTH / 2;
static const double TABLE_Y = 0; //-TABLE_WIDTH/2;

void cleanEnvironment(moveit_visual_tools::MoveItVisualToolsPtr visual_tools_)
{
  visual_tools_->cleanupCO(SUPPORT_SURFACE3_NAME);
}

void createEnvironment(moveit_visual_tools::MoveItVisualToolsPtr visual_tools_)
{
  SUPPORT_SURFACE3_NAME = "table";

  visual_tools_->cleanupCO(SUPPORT_SURFACE3_NAME);

  // Tables                          x,         y,          angle, width,       height,       depth,       name
  visual_tools_->publishCollisionTable(TABLE_X,   TABLE_Y,    0,     TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, SUPPORT_SURFACE3_NAME, rviz_visual_tools::BLACK); // andy table

  ROS_INFO_STREAM_NAMED("pick_place:", "SUPPORT_SURFACE is " << SUPPORT_SURFACE3_NAME);
}

double getTableHeight(double floor_offset)
{
  return TABLE_HEIGHT + floor_offset; // + BLOCK_SIZE / 2;
}

void getTableWidthRange(double &y_min, double &y_max)
{
  y_min = TABLE_Y - TABLE_WIDTH / 2;
  y_max = TABLE_Y + TABLE_WIDTH / 2;
}

void getTableDepthRange(double &x_min, double &x_max)
{
  x_min = TABLE_X - TABLE_DEPTH / 2;
  x_max = TABLE_X + TABLE_DEPTH / 2;
}

} // namespace

#endif
