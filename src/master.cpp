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
#include "niceReadXYZ.cpp"
#include <ros/ros.h>
#include "diff.cpp"
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



  /* statics */
  static string kind = "discrete";
  static int DIM = 3;
  static double EXTRA_TIME = 0; //sec
  static double MAIN_TIME;

  //times
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
  dataFile<<DIM<<endl;

  /* Start time*/
  gettimeofday (&startwtime, NULL);

  /*  Load data */
  // mat y_desired = loadData("data/DataPos_1D.txt");
  // mat dy_desired = loadData("data/DataVel_1D.txt");
  // mat ddy_desired = loadData("data/DataAccel_1D.txt");

  mat y_test = loadData("Collector/PosX.txt");
  int data_size = y_test.n_cols;
  dataFile<<"Size"<<data_size<<endl;

  mat y_desired(DIM,data_size), dy_desired(DIM,data_size), ddy_desired(DIM,data_size);

  for (int d = 0; d < DIM; d++)
  {
      y_desired.row(d) = (loadData("Collector/PosX.txt"));
      dy_desired.row(d) = (loadData("Collector/VelX.txt"));
      ddy_desired.row(d) = (loadData("Collector/AccelX.txt"));

  }

  infoFile <<"Load Data" << endl;

  /* Load Time*/
  vec timed = loadTime(data_size);

  infoFile <<"Load Time" << endl;

  /* Set time vars*/
  double ts = timed(3) - timed(2);
  MAIN_TIME = timed(timed.n_rows-1);

  //  --or-----------
  /*  diddMat to get velocity n' accel */

  mat y_or = niceReadXYZ("data/DataPos_3Dt.txt");
  mat dy_or = diffMat(data_size, DIM, ts, y_or);
  mat ddy_or = diffMat(data_size, DIM, ts, dy_or);

  vec goal(DIM);
  goal = y_desired.col(data_size-1);

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
  }

  infoFile <<"Training DONE" << endl;

  /*  Solution  */
  static double TOTAL_DURATION = MAIN_TIME + EXTRA_TIME;
  int extra_samples = EXTRA_TIME/ts;

  infoFile <<"Solution Start" << endl;
  infoFile <<"Total Duration"<<TOTAL_DURATION<< endl;
  infoFile <<"Extra Time" <<EXTRA_TIME<< endl;

  /* Controller inits*/
  // copController copC;

  /*init_solution for each dmp*/
  int i  = 0;
  double t = 0;
  mat f_prev(3,DIM);

  for (int d = 0; d < DIM; d++)
  {
    f_prev.col(d) = dmp[d].init_solution_dt(goal[d], cs, MAIN_TIME,  extra_samples, t, i);
  }
   /* fix sigmoid*/
   mat tsig = linspace(-MAIN_TIME-10,EXTRA_TIME,data_size+extra_samples);
   mat sig = 1-1/(1+exp(-tsig));

  for( t = ts ; t < (MAIN_TIME + EXTRA_TIME + ts); t = t + ts)
  {

     i++;
     for (int d = 0; d < DIM; d++)
     {
       f_prev.col(d) = dmp[d].run_solution_dt(t, goal[d],  ts, sig(i), i, f_prev(0,d), f_prev(1,d), f_prev(2,d));
     }
     // every dmp[d].y xDIM -> [x,y,z] then Q

     //controller.
  }

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
  }
  // for (int d = 0; d < DIM; d++)
  // {
  //   dmp[d].run_solution(goal[d], cs, ts, MAIN_TIME, EXTRA_TIME, extra_samples);
  // }


  // /* Reverse Solution*/
  // for (int d = 0; d < DIM; d++)
  // {
  //   dmp[d].run_reverse_solution(goal[d], cs, ts, MAIN_TIME, EXTRA_TIME, extra_samples);
  // }

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

  infoFile.close();
  warnFile.close();
  errorFile.close();
  dataFile.close();

  return 0;

}
