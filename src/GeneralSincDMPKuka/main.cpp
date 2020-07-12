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

using namespace std;
using namespace arma;

int main(int argc, char const *argv[])
{
  /* This code is based on DMPs - General DMP - easy to reverse it
              Despina - Ekaterini Argiropoulos
                    June 2020
              C++ Develop for Kuka Implementaion
  */

  /* statics */
  static string kind = "discrete";
  static int DIM = 1;
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

  /*  Load data */
  mat y_desired = loadData("data/DataPos_1D.txt");
  mat dy_desired = loadData("data/DataVel_1D.txt");
  mat ddy_desired = loadData("data/DataAccel_1D.txt");

  int data_size = y_desired.n_cols;

  /* Load Time*/
  vec timed = loadTime(data_size);

  /* Start time*/
  gettimeofday (&startwtime, NULL);

  /** Set time vars*/
  double ts = timed(3) - timed(2);
  MAIN_TIME = timed(timed.n_rows-1);

  /*  DataFormer to get velocity n' accel */
  vec goal(DIM);
  goal = y_desired.col(data_size-1);

   /* Generate Canonical Structure */
  CanonicalStructure cs(data_size);
  cs.generateFx(kind, timed);

  /*  Init DPM  */
  GDMP dmp[DIM];

  /*  Training  */
  for (int d = 0; d < DIM; d++)
  {
    dmp[d].training(y_desired.row(d),dy_desired.row(d),ddy_desired.row(d),timed);
  }

  /*  Solution  */
  static double TOTAL_DURATION = MAIN_TIME + EXTRA_TIME;
  int extra_samples = EXTRA_TIME/ts;

  for (int d = 0; d < DIM; d++)
  {
    dmp[d].run_solution(goal[d], cs, ts, MAIN_TIME, EXTRA_TIME, extra_samples);
  }


  /* Reverse Solution*/
  for (int d = 0; d < DIM; d++)
  {
    dmp[d].run_reverse_solution(goal[d], cs, ts, MAIN_TIME, EXTRA_TIME, extra_samples);
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
