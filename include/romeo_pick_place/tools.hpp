#ifndef TOOLS_H
#define TOOLS_H

#include <ros/ros.h>

/**
   * @brief promptUserAction
   * @param obj_name object name to perform action
   * @param obj_count total number of objects
   * @return action to do*/
  int promptUserAction(const std::string obj_name, const int obj_count)
  {
    ROS_INFO_STREAM_NAMED("pick_place","What do you want me to do with the object "
                          << obj_name << " out of " << obj_count << " detected objects? "
                          << "Possible actions are: \n"
                          << " 0-process the next object, \n"
                          //<< " 1-approach the hand to the object, \n"
                          << " p - pick the object, \n"
                          //<< " 3-move the hand with the object, \n"
                          << " 4-release the hand, \n"
                          << " z - move the hand to the zero pose, \n"
                          << " i - move the hand to the init pose, \n"
                          << " q -exit, \n"
                          << " l - remove all objects and create a virtual object for the left hand, \n"
                          << " r - remove all objects and create a virtual object for the right hand, \n"
                          << " d - detect objects, \n"
                          << " n - next object, \n"
                          << " t - start goal space exploration, \n"
                          << " c - clean the scene, \n"
                          << " h - change the active hand, \n"
                          << " right/left/up/down - to move the virtual object"
                          << " your choise is "
                          );
    /*int in =0;
    std::cin >> in;*/
    char ascii;
    std::cin >> ascii;
    int in = (int) ascii;
    //std::cout << "**************" << in << std::endl;
    return in;
  }

  bool promptUserQuestion(const std::string command)
  {
    ROS_INFO_STREAM_NAMED("pick_place",command);
    char input; // used for prompting yes/no
    std::cin >> input;
    if( input == 'n' )
      return false;

    return true;
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
