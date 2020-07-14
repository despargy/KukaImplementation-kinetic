#include "autharl_core/robot/ros_model.h"
#include "autharl_core/robot/robot_sim.h"
#include <autharl_core/lwr_robot/robot.h>
#include <controller/compliant_controller.h>
#include <controller/training_controller.h>
#include "logger.h"

int main(int argc, char const *argv[]) {

  ros::init(argc, argv, "despar dmp test");

  // init n' creation of robot model
  auto robot_model = std::make_shared<arl::robot::ROSModel>();
  std::shared_ptr<arl::robot::Robot> robot;
  robot.reset(new arl::lwr::Robot(robot_model));
  ROS_INFO_STREAM("Robot has been created successfully")

  // NodeHandle
  //
  track_controller = tr
  return 0;
}
