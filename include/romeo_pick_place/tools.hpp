#ifndef TOOLS_H
#define TOOLS_H

#include <ros/ros.h>

/**
   * @brief promptUserAction
   * @return action to do*/
  int promptUserAction()
  {
    ROS_INFO_STREAM_NAMED("pick_place","Possible actions are: \n"
                          << " a - plan grasping the object, \n"
                          << " f - plan grasping the object with all poses, \n"
                          << " e - execute the planned action, \n"

                          << " u - reach the object and grasp, \n"
                          << " x - reach the object pregrasp pose, \n"
                          << " y - reach the object from top, \n"
                          << " w - reach the init pose, \n"

                          << " g - pick the object, \n"
                          << " p - place the object, \n"

                          << " v - show the pregrasp pose, \n"
                          << " i - move the hand to the init pose, \n"
                          << " z - move the hand to the zero pose, \n"
                          << " o - open hand, \n"
                          << " c - close hand, \n"
                          << " m - move the head to look down, \n"

                          << " t - test the goal space, \n"

                          << " l - remove all objects and create a virtual object for the left hand, \n"
                          << " r - remove all objects and create a virtual object for the right hand, \n"
                          << " s - clean the scene, \n"
                          << " d - detect objects, \n"
                          << " n - process the next object, \n"
                          << " h - change the active hand, \n"
                          << " q - exit, \n"
                          << " right/left/up/down - to move the virtual object"
                          << " your choise is "
                          );
    char ascii;
    std::cin >> ascii;
    int in = (int) ascii;
    return in;
  }

  bool promptUserQuestion(const std::string command)
  {
    ROS_INFO_STREAM_NAMED("pick_place",command);
    char input;
    std::cin >> input;
    if( input == 'n' ) // used for yes/no
      return false;

    return true;
  }

  std::string promptUserQuestionString()
  {
    ROS_INFO_STREAM_NAMED("pick_place","Possible actions are: \n"
                          << " a - plan grasping the object, \n"
                          << " f - plan grasping the object with all poses, \n"
                          << " e - execute the planned action, \n"

                          << " u - reach the object and grasp, \n"
                          << " x - reach the object pregrasp pose, \n"
                          << " y - reach the object from top, \n"
                          << " w - reach the init pose, \n"

                          << " g - pick the object, \n"
                          << " p - place the object, \n"

                          << " v - show the pregrasp pose, \n"
                          << " i - move the hand to the init pose, \n"
                          << " z - move the hand to the zero pose, \n"
                          << " o - open hand, \n"
                          << " c - close hand, \n"
                          << " m - move the head to look down, \n"

                          << " test_pick - test the goal space for picking, \n"
                          << " test_reach - test the goal space for reaching, \n"

                          << " l - remove all objects and create a virtual object for the left hand, \n"
                          << " r - remove all objects and create a virtual object for the right hand, \n"
                          << " s - clean the scene, \n"
                          << " d - detect objects, \n"
                          << " n - process the next object, \n"
                          << " h - change the active hand, \n"
                          << " q - exit, \n"
                          << " right/left/up/down - to move the virtual object"
                          << " your choise is "
                          );
    std::string input;
    std::cin >> input;
    return input;
  }

  double promptUserValue(const std::string command)
  {
    ROS_INFO_STREAM_NAMED("pick_place",command);
    double input;
    std::cin >> input;
    return input;
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
#endif // TOOLS_H
