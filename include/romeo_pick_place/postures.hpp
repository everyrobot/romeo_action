#ifndef POSTURES_HPP
#define POSTURES_HPP

#include <stdlib.h>
#include <string>
#include <vector>

class Posture
{
public:
  Posture();

  void getPose(const std::string group_name);
  bool poseHeadDown();
  bool poseHandZero(const std::string end_eff, const std::string group);
  bool poseHandInit(const std::string &end_eff, const std::string &group, const std::string &arm);
  bool poseHandPregrasp(const std::string &end_eff, const std::string &group, const std::string &arm);

  std::vector<double> pose_head_down;
  std::vector<double> pose_hand_zero;
  std::vector<double> pose_arm_zero;
  std::vector<double> pose_arm_right_init;
  std::vector<double> pose_arm_left_init;
  std::vector<double> pose_arm_left_pregrasp;

  private:
    bool goToPose(const std::string group_name, std::vector<double> *pose);
};

#endif // POSTURES_HPP
