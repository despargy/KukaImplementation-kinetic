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

#include <kuka_implementation/collector.h>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <armadillo>


using namespace std;
using namespace arma;

Collector::Collector(const std::shared_ptr<arl::robot::Robot>& robot) :
  arl::robot::Controller(robot, "Collector")
{
  jnt_torques.resize(robot->model->getNrOfJoints());
}

void Collector::init()
{
  robot->setMode(arl::robot::Mode::TORQUE_CONTROL);
  myFile.open("Collector/posCollected.txt");

}

void Collector::update()
{
  // open file
   vec pos_got(7);
   pos_got = robot->getJointPosition().toArma();
   // for (int i = 0; i < 7; i++)
   //   myFile<<fd_filtered_ext[i]<<endl;
   this->myFile<<pos_got.t()<<endl;

  jnt_torques.setZero();
  robot->setJointTorque(jnt_torques);
  // if (27 == getchar()) // 27 is for ESC
  // {
  //   myFile.close();
  // }
}

void Collector::command()
{
  // robot->setJointTorque(jnt_torques);
}

bool Collector::stop()
{
  if (!ros::ok())
    myFile.close();

  return !ros::ok();
}
