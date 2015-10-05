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

#include <boost/program_options.hpp>
#include <romeo_pick_place/simplepickplace.hpp>

void parse_command_line(int argc, char ** argv, std::string &robot_name_, double &test_step_, bool &verbose_)
{
  std::string robot_name;
  double test_step;
  bool verbose;
  boost::program_options::options_description desc("Configuration");
  desc.add_options()
    ("help", "show this help message")
    ("robot_name", boost::program_options::value<std::string>(&robot_name)->default_value(robot_name_),
     "robot_name")
    ("test_step", boost::program_options::value<double>(&test_step)->default_value(test_step_),
     "test_step")
    ("verbose", boost::program_options::value<bool>(&verbose)->default_value(verbose_),
     "verbose")
    /*("depth_frame_id", boost::program_options::value<std::string>(&depth_frame_id)->default_value(m_depth_frame_id),
     "depth_frame_id")*/
    ;
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);
  robot_name_ = vm["robot_name"].as<std::string>();
  test_step_ = vm["test_step"].as<double>();
  verbose_ = vm["verbose"].as<bool>();
  ROS_INFO_STREAM("robot name is " << robot_name_);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return ;
  }
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "pick_place_moveit");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  /*if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }*/
  //verbose = true;

  std::string robot_name("romeo");
  double test_step(0.0);
  parse_command_line(argc, argv, robot_name, test_step, verbose);

  srand (time(NULL));

  // Start the pick place node
  romeo_pick_place::SimplePickPlace server(robot_name, test_step, verbose);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
