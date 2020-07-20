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
#include "readQ.cpp"
#include <ros/ros.h>
#include "diff.cpp"
#include <thread>
#include <autharl_core>
#include <lwr_robot/robot.h>


using namespace std;
using namespace arma;

/* This code is based on DMPs - General DMP - easy to reverse it
            Despina - Ekaterini Argiropoulos
                  June 2020
            C++ Develop for Kuka Implementaion
*/

int main(int argc, char** argv)
{

  /* statics */
  static string kind = "discrete";
  static int DIM = 7;
  static double EXTRA_TIME = 0; //sec
  static double MAIN_TIME;

  /* Count time duration of program */
  struct timeval startwtime, endwtime;
  double seq_time;
  time_t ttime = time(0);
  char* dt = ctime(&ttime);

  /* Start time*/
  gettimeofday (&startwtime, NULL);

  /* Logs */
  ofstream infoFile, warnFile, errorFile, dataFile;
  infoFile.open("Logs/info.log");
  warnFile.open("Logs/warn.log");
  errorFile.open("Logs/error.log");
  dataFile.open("Logs/data.log");

  infoFile << dt << endl;

  infoFile <<"Load Data: START" << endl;
  mat y_desired = readQ();
  infoFile <<"Load Data: DONE" << endl;

  // ofstream myFile;
  // std::ostringstream oss, ossd, ossdd;
  //
  // oss << "CHECK/yreadQ.txt";
  //
  // myFile.open((oss.str()).c_str());
  // myFile<<y_desired<<endl;
  // myFile.close();

  int DataSize = y_desired.n_cols;
  dataFile<<kind<<endl;
  dataFile<<DIM<<"x"<<DataSize<<endl;

  /* Set goal */
  vec goal(DIM);
  goal = y_desired.col(DataSize-1);

  /* Initialize the ROS node */
  ros::init(argc, argv, "get_desired_trajectory");
  ros::NodeHandle n;
  auto model = std::make_shared<arl::robot::ROSModel>();
  auto robot = std::make_shared<arl::robot::RobotSim>(model, 1e-3);

  // auto robot = std::make_shared<arl::lwr::Robot>(model);
  auto rviz = std::make_shared<arl::viz::RosStatePublisher>(robot);
  std::thread rviz_thread(&arl::viz::RosStatePublisher::run, rviz);

  /* Initialize joints */
  vec initial_config = y_desired.col(0); //{1.6968 ,   0.5681,   -1.1651 ,  -1.6293  ,  0.3011  ,  1.1462 ,   1.7304};

  robot->setMode(arl::robot::Mode::POSITION_CONTROL);
  robot->setJointTrajectory(initial_config, 10);

  /* Load Time*/
  double DTS = robot->cycle;
  vec timed(DataSize); // = loadTime(DataSize);

  infoFile <<"Load Time: START" << endl;
  for (int l = 0; l<DataSize; l++)
    timed(l) = l*DTS;
  infoFile <<"Load Time: DONE" << endl;

  /* Set time vars*/
  double ts = timed(3) - timed(2);
  MAIN_TIME = timed(DataSize-1);

  /*  diddMat to get velocity n' accel */
  mat dy_desired = diffMat(DataSize, DIM, ts, y_desired);
  mat ddy_desired = diffMat(DataSize, DIM, ts, dy_desired);

   /* Generate Canonical Structure */
  CanonicalStructure cs(DataSize);
  cs.generateFx(kind, timed);

  infoFile <<"Canonical Structure : DONE" << endl;

  /*  Init DPM  */
  GDMP dmp[DIM];
  infoFile <<"Initialize DMPs: DONE" << endl;

  //////////////////////////////////////////////////////////////////////////////

  /*  Training  */
  infoFile <<"Training of DMPs: START" << endl;

  for (int d = 0; d < DIM; d++)
  {
    dmp[d].training(y_desired.row(d),dy_desired.row(d),ddy_desired.row(d),timed);
    dataFile<<"id:"<<d<<'\t'<<"BFs = "<<dmp[d].BFs<<endl;
  }

  infoFile <<"Training of DMPs: DONE" << endl;

//////////////////////////////////////////////////////////////////////////////

  /*  Solution  */
  static double TOTAL_DURATION = MAIN_TIME + EXTRA_TIME;
  int extra_samples = ceil(EXTRA_TIME/ts);

  infoFile <<"Solution Start " << endl;
  infoFile <<"Total Duration = "<<TOTAL_DURATION<< endl;
  infoFile <<"Extra Time = " <<EXTRA_TIME<< endl;

  int i  = 0;
  double t = 0;
  mat f_prev(3,DIM);

  /*init_solution for each dmp*/
  infoFile <<"InitSolution of DMPs: START" << endl;

  for (int d = 0; d < DIM; d++)
  {
    f_prev.col(d) = dmp[d].init_solution_dt(goal[d], cs, MAIN_TIME,  extra_samples, t, i);
  }
  infoFile <<"InitSolution of DMPs: DONE" << endl;

   /* fix sigmoid*/
   mat tsig = linspace(-MAIN_TIME-10,EXTRA_TIME,DataSize+extra_samples);
   mat sig = 1-1/(1+exp(-tsig));

   // vecs to help for set n' get position
   vec commanded_pos(7), measured_pos(7);
   commanded_pos = initial_config;
   measured_pos = initial_config;

   // open file to store messures
   ofstream messuresFile, timeFile;

   messuresFile.open("Messures/robotJointPositions.txt");
   timeFile.open("Messures/robotJointTime.txt");

   measured_pos = robot->getJointPosition().toArma();
   messuresFile<<measured_pos<<endl;
   timeFile<<t<<endl;

   // Run Solution for each t
  infoFile <<"Solution of DMPs: START" << endl;
  for( t = ts ; t < (MAIN_TIME + EXTRA_TIME ); t = t + ts)
  {
    // get and store robot messures
     measured_pos = robot->getJointPosition().toArma();

     messuresFile<<measured_pos<<endl;
     timeFile<<t<<endl;

     i++;

     // call for each DIM = 7 based on Quat
     for (int d = 0; d < DIM-1; d++)
     {
       f_prev.col(d) = dmp[d].run_solution_dt(t, goal[d],  ts, sig(i), i, f_prev(0,d), f_prev(1,d), f_prev(2,d));
       commanded_pos(d) = dmp[d].y(i);
     }

     // // set new position
     robot->setJointPosition(commanded_pos);
     //
     // // wait for next robot cycle
     robot->waitNextCycle();
  }
  infoFile <<"Solution of DMPs: DONE" << endl;

  // close file stored messures from robot
  messuresFile.close();
  timeFile.close();

  /* Save results*/
  for (int d = 0; d < DIM; d++)
  {
    ofstream myFile;
    std::ostringstream oss, ossd, ossdd;

    oss << "CHECK/y" << d <<".log";

    myFile.open((oss.str()).c_str());
    myFile<<dmp[d].y<<endl;
    myFile.close();

    ossd << "CHECK/dy" << d <<".log";

    myFile.open((ossd.str()).c_str());
    myFile<<dmp[d].dy<<endl;
    myFile.close();

    ossdd << "CHECK/ddy" << d <<".log";

    myFile.open((ossdd.str()).c_str());
    myFile<<dmp[d].ddy<<endl;
    myFile.close();

    oss << "CHECK/w" << d <<".log";

    myFile.open((oss.str()).c_str());
    myFile<<dmp[d].w<<endl;
    myFile.close();

    oss << "CHECK/psi" << d <<".log";

    myFile.open((oss.str()).c_str());
    myFile<<dmp[d].psi<<endl;
    myFile.close();

    oss << "CHECK/c" << d <<".log";

    myFile.open((oss.str()).c_str());
    myFile<<dmp[d].c<<endl;
    myFile.close();
  }

//////////////////////////////////////

  cout<<"FORWARD SOLUTION DONE"<<endl;

  // /* Initialize joints */
  // vec Rinitial_config = y_desired.col(DataSize-1);
  //
  // robot->setMode(arl::robot::Mode::POSITION_CONTROL);
  // robot->setJointTrajectory(Rinitial_config, 10);
///////////////////////////////////////////
  char c = 'e';
  cout<<"before gotit"<<endl;

  if ('R' == getchar()) // 27 is for ESC
  {
    cout<<"gotit"<<endl;
  }
///////////////////////////////////////////

  /* Reverse Solution*/
  i  = 0;
  t = 0;
  mat Rf_prev(3,DIM);

  /*init_Rsolution for each dmp*/
  infoFile <<"RInitSolution of DMPs: START" << endl;

  for (int d = 0; d < DIM; d++)
  {
    Rf_prev.col(d) = dmp[d].init_Rsolution_dt(goal[d], cs, MAIN_TIME,  extra_samples, t, i);
  }
  infoFile <<"InitSolution of DMPs: DONE" << endl;

   /* fix sigmoid*/
   tsig = linspace(-MAIN_TIME-10,EXTRA_TIME,DataSize+extra_samples);
   sig = 1-1/(1+exp(-tsig));

   // vecs to help for set n' get position
   vec Rcommanded_pos(7), Rmeasured_pos(7);
   Rcommanded_pos.zeros();
   Rmeasured_pos.zeros();

   // open file to store messures
   ofstream RmessuresFile, RtimeFile;

   RmessuresFile.open("Messures/RrobotJointPositions.txt");
   RtimeFile.open("Messures/RrobotJointTime.txt");

   Rmeasured_pos = robot->getJointPosition().toArma();
   RmessuresFile<<Rmeasured_pos<<endl;
   RtimeFile<<t<<endl;

   // Run RSolution for each t
  infoFile <<"RSolution of DMPs: START" << endl;

  for( t = ts ; t < (MAIN_TIME + EXTRA_TIME ); t = t + ts)
  {
     // get and store robot messures
     Rmeasured_pos = robot->getJointPosition().toArma();

     RmessuresFile<<Rmeasured_pos<<endl;
     RtimeFile<<t<<endl;

     i++;

     for (int d = 0; d < DIM; d++)
     {
       Rf_prev.col(d) = dmp[d].run_Rsolution_dt(t, goal[d],  ts, sig(i), i, Rf_prev(0,d), Rf_prev(1,d), Rf_prev(2,d));
       Rcommanded_pos(d) = dmp[d].Ry(i);
     }

    // set new position
    robot->setJointPosition(Rcommanded_pos);

    // wait for next robot cycle
    robot->waitNextCycle();
  }
  infoFile <<"RSolution of DMPs: DONE" << endl;

  // close file stored messures from robot
  RmessuresFile.close();

  /* Save Rresults*/
  for (int d = 0; d < DIM; d++)
  {
    ofstream myFile;
    std::ostringstream oss, ossd, ossdd;

    oss << "CHECK/Ry" << d <<".log";

    myFile.open((oss.str()).c_str());
    myFile<<dmp[d].Ry<<endl;
    myFile.close();

    ossd << "CHECK/Rdy" << d <<".log";

    myFile.open((ossd.str()).c_str());
    myFile<<dmp[d].Rdy<<endl;
    myFile.close();

    ossdd << "CHECK/Rddy" << d <<".log";

    myFile.open((ossdd.str()).c_str());
    myFile<<dmp[d].Rddy<<endl;
    myFile.close();
  }
  cout<<"REVERSE SOLUTION DONE"<<endl;

  /* End time*/
  gettimeofday (&endwtime, NULL);
  seq_time = (double)((endwtime.tv_usec - startwtime.tv_usec)/1.0e6 + endwtime.tv_sec - startwtime.tv_sec);
  infoFile<<"GDMP clock time = "<<seq_time<<endl;

  // close Log files
  infoFile.close();
  warnFile.close();
  errorFile.close();
  dataFile.close();

  cout<<"Thank you \n \t Bye Bye"<<endl;

  return 0;

}
