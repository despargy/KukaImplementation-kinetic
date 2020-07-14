#include <ros/ros.h>
#include <thread>

#include <autharl_core/controller/gravity_compensation.h>

#include <autharl_core/robot/robot_sim.h>
#include <lwr_robot/robot.h>

#include <autharl_core/viz/ros_state_publisher.h>
#include <autharl_core/robot/ros_model.h>

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "get_desired_trajectory");
  ros::NodeHandle n;

  // Create the robot after you have launch the URDF on the parameter server
  auto model = std::make_shared<arl::robot::ROSModel>();

  // Create a simulated robot, use can use a real robot also
  // auto robot = std::make_shared<arl::robot::RobotSim>(model, 1e-3);
  auto robot = std::make_shared<arl::lwr::Robot>(model);

  // Create a visualizater for see the result in rviz
  auto rviz = std::make_shared<arl::viz::RosStatePublisher>(robot);

  // Create the joint trajectory controller
  auto gravity_compensation = std::make_shared<arl::controller::GravityCompensation>(robot);

  std::thread rviz_thread(&arl::viz::RosStatePublisher::run, rviz);

  // Run the trajectory controller
  gravity_compensation_position->run();


  ros::spin();
  rviz_thread.join();

  // thread to collect Data

  // store them by GravityX.txt , GravitY.txt  GravityZ.txt under /data folder

  return 0;
}
