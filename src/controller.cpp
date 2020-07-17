#include <ros/ros.h>
#include "std_msgs/String.h"
#include <armadillo>
#include <iostream>
#include <autharl_core/robot/controller.h>

using namespace arma;

class copController : public arl::robot::Controller
{
  public:
      copController(std::shared_ptr<Robot> r) :
      arl::robot::Controller(r, 'copController'){};
      ~copController(){};
      void solutionCallback(const std_msgs::String::ConstPtr& msg);
      void action(){};
      void working(){};
};

// void copController::solutionCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }
//
// void copController::working()
// {
//   // ros::init(argc, argv, "listener");
//   ros::NodeHandle n;
//   ros::Subscriber sub = n.subscribe("chatter", 1000, &Controller::solutionCallback, this);
//   ros::spin();
// }
