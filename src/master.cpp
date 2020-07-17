#include <iostream>
#include <string>
#include <armadillo>
#include <stdio.h>
#include <stdlib.h>
#include "CanonicalStructure.cpp"
#include "dataFormer.cpp"
#include "GDMP.cpp"
#include "loadData.cpp"
#include "loadTime.cpp"
#include "LowPassFilter.hpp"
// #include "controller.cpp"
#include "readQ.cpp"
#include <ros/ros.h>
#include "diff.cpp"

// AUTh-ARL core
#include <thread>
#include <autharl_core>
#include <lwr_robot/robot.h>
// #include "controller.cpp"

/* includes from example*/
// #include <thread>
//
// #include <autharl_core/controller/gravity_compensation.h>
//
// #include <autharl_core/robot/robot_sim.h>
// // #include <lwr_robot/robot.h>
//
// #include <autharl_core/viz/ros_state_publisher.h>
// #include <autharl_core/robot/ros_model.h>

using namespace std;
using namespace arma;

/* This code is based on DMPs - General DMP - easy to reverse it
            Despina - Ekaterini Argiropoulos
                  June 2020
            C++ Develop for Kuka Implementaion
*/

int main(int argc, char** argv)
{

  // Initialize the ROS node
  ros::init(argc, argv, "get_desired_trajectory");
  ros::NodeHandle n;
  auto model = std::make_shared<arl::robot::ROSModel>();
  auto robot = std::make_shared<arl::robot::RobotSim>(model, 1e-3);

  // auto robot = std::make_shared<arl::lwr::Robot>(model);
  auto rviz = std::make_shared<arl::viz::RosStatePublisher>(robot);

  std::thread rviz_thread(&arl::viz::RosStatePublisher::run, rviz);
  vec initial_config = {1.6968 ,   0.5681,   -1.1651 ,  -1.6293  ,  0.3011  ,  1.1462 ,   1.7304};
  robot->setMode(arl::robot::Mode::POSITION_CONTROL);
  robot->setJointTrajectory(initial_config, 10);

  /* statics */
  static string kind = "discrete";
  static int DIM = 7;
  static double EXTRA_TIME = 0; //sec
  static double MAIN_TIME;
  int data_size = 7879;

  // Count time duration of program
  struct timeval startwtime, endwtime;
  double seq_time;
  time_t ttime = time(0);
  char* dt = ctime(&ttime);

  // Logs
  ofstream infoFile, warnFile, errorFile, dataFile;
  infoFile.open("Logs/info.log");
  warnFile.open("Logs/warn.log");
  errorFile.open("Logs/error.log");
  dataFile.open("Logs/data.log");

  infoFile<<"Test info.log"<<endl;
  infoFile <<dt << endl;

  dataFile<<kind<<endl;
  dataFile<<DIM<<"x"<<data_size<<endl;

  /* Start time*/
  gettimeofday (&startwtime, NULL);

  /*  Load data */
  // mat y_desired = loadData("data/DataPos_1D.txt");
  // mat dy_desired = loadData("data/DataVel_1D.txt");
  // mat ddy_desired = loadData("data/DataAccel_1D.txt");

  // mat y_test = loadData("Collector/pos_got.log");
  // int data_size = y_test.n_cols;
  // dataFile<<"Size"<<data_size<<endl;

  // mat y_desired(DIM,data_size), dy_desired(DIM,data_size), ddy_desired(DIM,data_size);

  // for (int d = 0; d < DIM; d++)
  // {
  //     y_desired.row(d) = (loadData("Collector/PosX.txt"));
  //     dy_desired.row(d) = (loadData("Collector/VelX.txt"));
  //     ddy_desired.row(d) = (loadData("Collector/AccelX.txt"));
  //
  // }


  mat y_desired(DIM,data_size), dy_desired(DIM,data_size), ddy_desired(DIM,data_size);
  infoFile <<"Load Data" << endl;
  y_desired = readQ();

    ofstream myFile;
    std::ostringstream oss, ossd, ossdd;

    oss << "CHECK/yreadQ.txt";

    myFile.open((oss.str()).c_str());
    myFile<<y_desired<<endl;
    myFile.close();

  vec goal(DIM);
  goal = y_desired.col(data_size-1);

  /* Load Time*/
  double DTS = robot->cycle;
  vec timed(data_size); // = loadTime(data_size);

  for (int l = 0; l<data_size; l++)
    timed(l) = l*DTS;

  /* Set time vars*/
  double ts = timed(3) - timed(2);
  MAIN_TIME = timed(data_size-1);

  infoFile <<"Load Time" << endl;

  /*  diddMat to get velocity n' accel */
  dy_desired = diffMat(data_size, DIM, ts, y_desired);
  ddy_desired = diffMat(data_size, DIM, ts, dy_desired);

cout<<y_desired.n_cols<<endl;
cout<<dy_desired.n_cols<<endl;
cout<<ddy_desired.n_cols<<endl;

   /* Generate Canonical Structure */
  CanonicalStructure cs(data_size);
  cs.generateFx(kind, timed);

  infoFile <<"Generate Canonical Structure" << endl;

  /*  Init DPM  */
  GDMP dmp[DIM];
  infoFile <<"Init DMP" << endl;

  /*  Training  */
  for (int d = 0; d < DIM; d++)
  {
    dmp[d].training(y_desired.row(d),dy_desired.row(d),ddy_desired.row(d),timed);
    dataFile<<"id:"<<d<<'\t'<<"BFs = "<<dmp[d].BFs<<endl;

  }

  infoFile <<"Training DONE" << endl;
  /*  Solution  */
  static double TOTAL_DURATION = MAIN_TIME + EXTRA_TIME;
  int extra_samples = EXTRA_TIME/ts;

  infoFile <<"Solution Start " << endl;
  infoFile <<"Total Duration = "<<TOTAL_DURATION<< endl;
  infoFile <<"Extra Time = " <<EXTRA_TIME<< endl;

  int i  = 0;
  double t = 0;
  mat f_prev(3,DIM);

  /*init_solution for each dmp*/
  for (int d = 0; d < DIM; d++)
  {
    f_prev.col(d) = dmp[d].init_solution_dt(goal[d], cs, MAIN_TIME,  extra_samples, t, i);
  }

   /* fix sigmoid*/
   mat tsig = linspace(-MAIN_TIME-10,EXTRA_TIME,data_size+extra_samples);
   mat sig = 1-1/(1+exp(-tsig));

   // vecs to help for set n' get position
   vec commanded_pos(7), measured_pos(7);
   commanded_pos.zeros();
   measured_pos.zeros();

   // open file to store messures
   ofstream messuresFile;
   messuresFile.open("Messures/robotJointPositions.txt");

   // Run Solution for each t
  for( t = ts ; t < (MAIN_TIME + EXTRA_TIME + ts); t = t + ts)
  {
    // get and store robot messures
     measured_pos = robot->getJointPosition().toArma();
     messuresFile<<measured_pos<<endl;

     i++;

     // call for each DIM = 7 based on Quat
     for (int d = 0; d < DIM; d++)
     {
       f_prev.col(d) = dmp[d].run_solution_dt(t, goal[d],  ts, sig(i), i, f_prev(0,d), f_prev(1,d), f_prev(2,d));
       commanded_pos(d) = dmp[d].y(i);
     }

     // assign position

     // set new position
     robot->setJointPosition(commanded_pos);

     // wait for next robot cycle
     robot->waitNextCycle();
  }

  // close file stored messures from robot
  messuresFile.close();

  for (int d = 0; d < DIM; d++)
  {
    ofstream myFile;
    std::ostringstream oss, ossd, ossdd;

    oss << "CHECK/y" << d <<".log";

    myFile.open((oss.str()).c_str());
    myFile<<dmp[d].y<<endl;
    myFile.close();

    // ossd << "CHECK/dy" << d <<".log";
    //
    // myFile.open((ossd.str()).c_str());
    // myFile<<dmp[d].dy<<endl;
    // myFile.close();
    //
    // ossdd << "CHECK/ddy" << d <<".log";
    //
    // myFile.open((ossdd.str()).c_str());
    // myFile<<dmp[d].ddy<<endl;
    // myFile.close();
  }


/* Reverse Solution*/
  i  = 0;
  t = 0;
  mat Rf_prev(3,DIM);

  for (int d = 0; d < DIM; d++)
  {
    Rf_prev.col(d) = dmp[d].init_Rsolution_dt(goal[d], cs, MAIN_TIME,  extra_samples, t, i);
  }
   /* fix sigmoid*/
   tsig = linspace(-MAIN_TIME-10,EXTRA_TIME,data_size+extra_samples);
   sig = 1-1/(1+exp(-tsig));

  for( t = ts ; t < (MAIN_TIME + EXTRA_TIME + ts); t = t + ts)
  {

     i++;
     for (int d = 0; d < DIM; d++)
     {
       Rf_prev.col(d) = dmp[d].run_Rsolution_dt(t, goal[d],  ts, sig(i), i, Rf_prev(0,d), Rf_prev(1,d), Rf_prev(2,d));
     }
     // every dmp[d].y xDIM -> [x,y,z] then Q

     //controller.
  }


  /* End time*/
  gettimeofday (&endwtime, NULL);
  seq_time = (double)((endwtime.tv_usec - startwtime.tv_usec)/1.0e6 + endwtime.tv_sec - startwtime.tv_sec);
  infoFile<<"GDMP clock time = "<<seq_time<<endl;

  // close Log files
  infoFile.close();
  warnFile.close();
  errorFile.close();
  dataFile.close();

  return 0;

}
