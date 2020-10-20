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
#include "filters_smooth.cpp"

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
    dmp[d].id = d;
    dmp[d].training(y_desired.row(d),dy_desired.row(d),ddy_desired.row(d),timed);
    dataFile<<"id:"<<d<<'\t'<<"BFs = "<<dmp[d].BFs<<endl;
  }

  infoFile <<"Training of DMPs: DONE" << endl;

//////////////////////////////////////////////////////////////////////////////
                  ////////////FORWARD//////////

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

   // Run Solution for each t
  infoFile <<"Solution of DMPs: START" << endl;
  for( t = ts ; t < (MAIN_TIME + EXTRA_TIME+ ts); t = t + ts)
  {

     i++;

     // call for each DIM = 7 based on Quat
     for (int d = 0; d < DIM; d++)
     {
       f_prev.col(d) = dmp[d].run_solution_dt(t, goal[d], ts,  i, f_prev(0,d), f_prev(1,d), f_prev(2,d));
     }

  }
  infoFile <<"Solution of DMPs: DONE" << endl;

  cout<<"FORWARD SOLUTION DONE"<<endl;

//////////////////////////////////////////////////////////////////////////////
                    ////////////REVERSE//////////

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


   // Run RSolution for each t
  infoFile <<"RSolution of DMPs: START" << endl;

  for( t = ts ; t < (MAIN_TIME + EXTRA_TIME + ts); t = t + ts)
  {

     i++;

     for (int d = 0; d < DIM; d++)
     {
       Rf_prev.col(d) = dmp[d].run_Rsolution_dt(t, goal[d],  ts, i, Rf_prev(0,d), Rf_prev(1,d), Rf_prev(2,d));
     }

  }
  infoFile <<"RSolution of DMPs: DONE" << endl;

  cout<<"REVERSE SOLUTION DONE"<<endl;

  //////////////////////////////////////////////////////////////////////////////
                    ////////////RESULTS//////////
cout<<"SAVE RESULTS START"<<endl;

    /* Save results*/
    for (int d = 0; d < DIM; d++)
    {
      ofstream myFile;
      std::ostringstream oss, ossd, ossdd, ossw, ossc, osspsi;

      oss << "CHECK/y" << d <<".log";

      myFile.open((oss.str()).c_str());
      for (int i = 0; i < dmp[d].NSamples; i++)
        myFile<<dmp[d].y(i)<<endl;
      myFile.close();

      ossd << "CHECK/dy" << d <<".log";

      myFile.open((ossd.str()).c_str());
      for (int i = 0; i < dmp[d].NSamples; i++)
        myFile<<dmp[d].dy(i)<<endl;
      myFile.close();

      ossdd << "CHECK/ddy" << d <<".log";

      myFile.open((ossdd.str()).c_str());
      for (int i = 0; i < dmp[d].NSamples; i++)
        myFile<<dmp[d].ddy(i)<<endl;
      myFile.close();

      ossw << "CHECK/w" << d <<".log";

      myFile.open((ossw.str()).c_str());
      for (int i = 0; i < dmp[d].BFs; i++)
        myFile<<dmp[d].w(i)<<endl;
      myFile.close();

      osspsi << "CHECK/psi" << d <<".log";

      myFile.open((osspsi.str()).c_str());
      for (int b = 0; b <dmp[d].BFs; b++)
      {
        for (int i = 0; i < dmp[d].NSamples+dmp[d].extra_train; i++)
          myFile<<dmp[d].psi(b,i)<<'\t';
        myFile<<endl;
      }
      myFile.close();

      ossc << "CHECK/c" << d <<".log";

      myFile.open((ossc.str()).c_str());
      for (int i = 0; i < dmp[d].BFs; i++)
        myFile<<dmp[d].c(i)<<endl;
      myFile.close();
    }

  //////////////////////////////////////
  /* Save Rresults*/
  for (int d = 0; d < DIM; d++)
  {
    ofstream myFile;
    std::ostringstream oss, ossd, ossdd;

    oss << "CHECK/Ry" << d <<".log";

    myFile.open((oss.str()).c_str());
    for (int i = 0; i < dmp[d].NSamples; i++)
      myFile<<dmp[d].Ry(i)<<endl;
    myFile.close();

    ossd << "CHECK/Rdy" << d <<".log";

    myFile.open((ossd.str()).c_str());
    for (int i = 0; i < dmp[d].NSamples; i++)
      myFile<<dmp[d].Rdy(i)<<endl;
    myFile.close();

    ossdd << "CHECK/Rddy" << d <<".log";

    myFile.open((ossdd.str()).c_str());
    for (int i = 0; i < dmp[d].NSamples; i++)
      myFile<<dmp[d].Rddy(i)<<endl;
    myFile.close();
  }

  cout<<"SAVE RESULTS DONE"<<endl;

  //////////////////////////////////////////////////////////////////////////////
                    ////////////FILTERING//////////

  Filter fsm(1);
  for (int d = 0; d < DIM; d++)
  {
    int window = round(2*dmp[d].tNyq/dmp[d].dt);
    int half_w = ceil(window/2)+1;
    int N = 2*half_w + dmp[d].y.n_cols ;

    cout<<window<<endl;
    cout<<half_w<<endl;
    cout<<dmp[d].y.n_cols<<endl;

    fsm.setNcoeffs(window);
    mat sig_in(1, N);
    mat sig_out(1,N);

    for (int i = 0; i < half_w ; i++)
    {
      sig_in(0,i) = dmp[d].y(0);
    }
    for (int i = 0; i < dmp[d].y.n_cols ; i++)
    {
      sig_in(0,i+half_w) = dmp[d].y(i);
    }
    for (int i = 0; i < half_w ; i++)
    {
      sig_in(0, dmp[d].y.n_cols + half_w + i) = dmp[d].y( dmp[d].y.n_cols - 1 );
    }

    sig_out = fsm.filterOffline(sig_in);

    ofstream myFile;
    std::ostringstream osssigout, oss_sm;
    osssigout << "CHECK/sig_in" << d <<".log";
    myFile.open((osssigout.str()).c_str());
    for (int i = 0; i < N; i++)
      myFile<<sig_in(i)<<endl;
    myFile.close();

    osssigout << "CHECK/sig_out" << d <<".log";
    myFile.open((osssigout.str()).c_str());
    for (int i = 0; i < N; i++)
      myFile<<sig_out(i)<<endl;
    myFile.close();

    dmp[d].y_smoothed.set_size(1,dmp[d].y.n_cols);
    for (int i = 0; i < dmp[d].y.n_cols ; i++)
    {
      dmp[d].y_smoothed(0,i) = sig_out(0,i+half_w);
    }

    oss_sm << "CHECK/y_smoothed" << d <<".log";
    myFile.open((oss_sm.str()).c_str());
    for (int i = 0; i < dmp[d].y.n_cols; i++)
      myFile<<dmp[d].y_smoothed(i)<<endl;
    myFile.close();

  }

  Filter Rfsm(1);
  for (int d = 0; d < DIM; d++)
  {
    int window = round(2*dmp[d].tNyq/dmp[d].dt);
    int half_w = ceil(window/2)+1;
    int N = 2*half_w + dmp[d].Ry.n_cols ;

    cout<<window<<endl;
    cout<<half_w<<endl;
    cout<<dmp[d].Ry.n_cols<<endl;

    Rfsm.setNcoeffs(window);
    mat sig_in(1, N);
    mat sig_out(1,N);

    for (int i = 0; i < half_w ; i++)
    {
      sig_in(0,i) = dmp[d].Ry(0);
    }
    for (int i = 0; i < dmp[d].Ry.n_cols ; i++)
    {
      sig_in(0,i+half_w) = dmp[d].Ry(i);
    }
    for (int i = 0; i < half_w ; i++)
    {
      sig_in(0, dmp[d].Ry.n_cols + half_w + i) = dmp[d].Ry( dmp[d].Ry.n_cols - 1 );
    }

    sig_out = Rfsm.filterOffline(sig_in);

    ofstream myFile;
    std::ostringstream osssigout, oss_sm;
    osssigout << "CHECK/Rsig_in" << d <<".log";
    myFile.open((osssigout.str()).c_str());
    for (int i = 0; i < N; i++)
      myFile<<sig_in(i)<<endl;
    myFile.close();

    osssigout << "CHECK/Rsig_out" << d <<".log";
    myFile.open((osssigout.str()).c_str());
    for (int i = 0; i < N; i++)
      myFile<<sig_out(i)<<endl;
    myFile.close();

    dmp[d].Ry_smoothed.set_size(1,dmp[d].Ry.n_cols);
    for (int i = 0; i < dmp[d].Ry.n_cols ; i++)
    {
      dmp[d].Ry_smoothed(0,i) = sig_out(0,i+half_w);
    }

    oss_sm << "CHECK/Ry_smoothed" << d <<".log";
    myFile.open((oss_sm.str()).c_str());
    for (int i = 0; i < dmp[d].Ry.n_cols; i++)
      myFile<<dmp[d].Ry_smoothed(i)<<endl;
    myFile.close();

  }

    //////////////////////////////////////////////////////////////////////////////
                      ////////////EXECUTION//////////


  /*  Forward Execution  */
  TOTAL_DURATION = MAIN_TIME + EXTRA_TIME;
  extra_samples = ceil(EXTRA_TIME/ts);

  infoFile <<"Solution Start " << endl;
  infoFile <<"Total Duration = "<<TOTAL_DURATION<< endl;
  infoFile <<"Extra Time = " <<EXTRA_TIME<< endl;

  i  = 0;
  t = 0;

  /*init_solution for each dmp*/
  infoFile <<"Execution of DMPs: START" << endl;


   // vecs to help for set n' get position
   vec commanded_pos(7), measured_pos(7);
   commanded_pos = initial_config;
   measured_pos = initial_config;

   // open file to store messures
   ofstream messuresFile, timeFile;

   messuresFile.open("Messures/robotJointPositions_s.txt");
   timeFile.open("Messures/robotJointTime_s.txt");

   measured_pos = robot->getJointPosition().toArma();
   messuresFile<<measured_pos<<endl;
   timeFile<<t<<endl;

   // Run Execution for each t
  infoFile <<"Execution of DMPs: START" << endl;
  for( t = ts ; t < (MAIN_TIME + EXTRA_TIME+ ts); t = t + ts)
  {
    // get and store robot messures
     measured_pos = robot->getJointPosition().toArma();

     messuresFile<<measured_pos<<endl;
     timeFile<<t<<endl;

     i++;

     // call for each DIM = 7 based on Quat
     for (int d = 0; d < DIM; d++)
     {
       commanded_pos(d) = dmp[d].y_smoothed(i);
     }

     // // set new position
     robot->setJointPosition(commanded_pos);
     //
     // // wait for next robot cycle
     robot->waitNextCycle();
  }
  infoFile <<"Execution of DMPs: DONE" << endl;

  // close file stored messures from robot
  messuresFile.close();
  timeFile.close();


  cout<<"FORWARD EXECUTION DONE"<<endl;

  ///////////////////////////////////////////
    // char c = 'e';
    // cout<<"before gotit"<<endl;
    //
    // if ('R' == getchar()) // 27 is for ESC
    // {
    //   cout<<"gotit"<<endl;
    // }
    //////////////////////////////////////////

  /* Reverse Execution*/
  i  = 0;
  t = 0;

  /*init_Rsolution for each dmp*/
  infoFile <<"R Execution of DMPs: START" << endl;

   // vecs to help for set n' get position
   vec Rcommanded_pos(7), Rmeasured_pos(7);
   Rcommanded_pos.zeros();
   Rmeasured_pos.zeros();

   // open file to store messures
   ofstream RmessuresFile, RtimeFile;

   RmessuresFile.open("Messures/RrobotJointPositions_s.txt");
   RtimeFile.open("Messures/RrobotJointTime_s.txt");

   Rmeasured_pos = robot->getJointPosition().toArma();
   RmessuresFile<<Rmeasured_pos<<endl;
   RtimeFile<<t<<endl;

   // Run RSolution for each t
  infoFile <<"R Execution of DMPs: START" << endl;

  for( t = ts ; t < (MAIN_TIME + EXTRA_TIME + ts); t = t + ts)
  {
     // get and store robot messures
     Rmeasured_pos = robot->getJointPosition().toArma();

     RmessuresFile<<Rmeasured_pos<<endl;
     RtimeFile<<t<<endl;

     i++;

     for (int d = 0; d < DIM; d++)
     {
       Rcommanded_pos(d) = dmp[d].Ry_smoothed(i);
     }

    // set new position
    robot->setJointPosition(Rcommanded_pos);

    // wait for next robot cycle
    robot->waitNextCycle();
  }
  infoFile <<"R Execution of DMPs: DONE" << endl;

  // close file stored messures from robot
  RmessuresFile.close();

  cout<<"REVERSE Execution DONE"<<endl;



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
