/*******************************************************************************
 * Copyright (c) 2016-2017 Automation and Robotics Lab, AUTh
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#ifndef KUKA_IMPLEMENTATION_CONTROLLER_H
#define KUKA_IMPLEMENTATION_CONTROLLER_H

#include <autharl_core/robot/controller.h>
#include <memory>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <armadillo>

/*!
 * \brief Implements a gravity compensation controller.
 *
 * This controller commands all of the joints of the robot with zero torques.
 *
 * \attention It assumes that the robot in use is torque controlled.
 *
 * Example of use:
 *
 * \code{.cpp}
 * #include <autharl_core/controllers/gravity_compensation.h>
 * #include <lwr_robot/robot/robot.h>
 * // ...
 * // Use the KUKA LWR4+
 * auto model  = std::make_shared<lwr::robot::Model>("/robot_description");
 * auto robot  = std::make_shared<lwr::robot::Robot>(model);
 * Collector controller(robot);
 * controller.run();
 * \endcode
 *
 * \ingroup RobotControllers
 */
 using namespace std;

class Collector : public arl::robot::Controller
{
public:
  explicit Collector(const std::shared_ptr<arl::robot::Robot>& robot);
  ofstream myFile;

protected:
  void init();
  void update();
  void command();
  bool stop();

private:
  Eigen::VectorXd jnt_torques;
};


#endif  // KUKA_IMPLEMENTATION_CONTROLLER_H
