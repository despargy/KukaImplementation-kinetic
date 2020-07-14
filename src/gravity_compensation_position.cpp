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

#include <kuka_implementation/gravity_compensation_position.h>
#include <ros/ros.h>

namespace arl
{
namespace controller
{
GravityCompensationPosition::GravityCompensationPosition(const std::shared_ptr<arl::robot::Robot>& robot) :
  arl::robot::Controller(robot, "Gravity Compensation")
{
  jnt_torques.resize(robot->model->getNrOfJoints());
}

void GravityCompensationPosition::init()
{
  robot->setMode(arl::robot::Mode::POSITION_CONTROL);
}

void GravityCompensationPosition::update()
{
  jnt_torques.setZero();
}

void GravityCompensationPosition::command()
{
  robot->setJointPosition(jnt_torques);
}

bool GravityCompensationPosition::stop()
{
  return !ros::ok();
}
}  // namespace controller
}  // namespace arl
