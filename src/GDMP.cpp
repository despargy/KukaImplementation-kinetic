#include <armadillo>
#include <iostream>
#include <cmath>
#include "LowPassFilter.hpp"
#include "LowPassFilter.cpp"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sigproc_lib/sigproc_lib.h>
#include "sigproc_lib/sigproc.h"

using namespace arma;
using namespace as64_;

class GDMP
{
public:

  int BFs, NSamples, extra_train, Ns, id;
  double tNyq, dt, Tend;
  double y0, dy0, g, y0d_hat, gd_hat, Ry0d_hat, Rgd_hat;
  double az, bz, K, D, k, ks, kt;

  mat c, h, psi, w;
  mat y, dy, ddy;
  mat Ry, Rdy, Rddy;

  GDMP();
  ~GDMP();
  void training(mat y_desired,mat dy_desired,mat ddy_desired, vec timed);
  mat find_reference_desired(mat y_desired,mat dy_desired,mat ddy_desired, vec timed);
  void calculate_centers(vec timed);
  void generate_weights(mat fp_des);
  void run_solution(double goal, CanonicalStructure cs, double ts, double main_time, double extra_time, int extra_samples);
  void run_reverse_solution(double goal, CanonicalStructure cs, double ts, double main_time, double extra_time, int extra_samples);
  double fNyquistFunc(double *original, int size, double fs, double T);
  vec run_solution_dt(double t_now, double goal, double ts, int i, double f_prev, double df_prev, double ddf_prev );
  vec init_solution_dt(double goal, CanonicalStructure cs, double main_time, int extra_samples, double t_now, int i );
  vec init_Rsolution_dt(double goal, CanonicalStructure cs, double main_time, int extra_samples, double t_now, int i);
  vec run_Rsolution_dt(double t_now, double goal, double ts, int i, double f_prev, double df_prev, double ddf_prev );

};

GDMP::GDMP()
{
  std::cout << "/* Constructor of GDMP */" << '\n';
}

GDMP::~GDMP()
{
  std::cout << "/* Deconstructor of GDMP */" << '\n';
}

void GDMP::training(mat y_desired,mat dy_desired,mat ddy_desired, vec timed)
{
  std::cout << "/* Start of Training */" << '\n';

  NSamples = y_desired.n_cols;
  dt = timed(3)-timed(2);
  Tend = timed(NSamples-1);
  extra_train = ceil(NSamples/2);

  /* Reference Desired*/
  mat mat_refD(size(y_desired));
  mat_refD = find_reference_desired(y_desired, dy_desired, ddy_desired, timed);

  /* Calculate centers h psi */
  calculate_centers(timed);

  /* Generate weights */
  generate_weights(mat_refD.row(0));

  std::cout << "/* End of Training */" << '\n';
}

mat GDMP::find_reference_desired(mat y_desired,mat dy_desired,mat ddy_desired, vec timed)
{
  cout<<"Reference Desired Checked"<<endl;

  ks = 1;
  kt = 1;
  y0 = y_desired(0);
  dy0 = dy_desired(0);
  g =  y_desired(NSamples-1);

  /* sig y_desired to 0*/
  mat tsig = linspace(-Tend-extra_train*dt,extra_train*dt,NSamples+extra_train);
  mat sig = 1-1/(1+exp(-0.08/dt*tsig));

  double y_scaled_ext[NSamples+extra_train], dy_ext[NSamples+extra_train], ddy_ext[NSamples+extra_train], timed_ext[NSamples+extra_train];

  for (int i = 0; i < NSamples; i++)
  {
    y_scaled_ext[i] = sig(i)*(y_desired[i] - y0);
    dy_ext[i] = sig(i)*dy_desired[i];
    ddy_ext[i] = sig(i)*ddy_desired[i];
    timed_ext[i] = timed(i);
  }

  for (int i = NSamples;i<NSamples+extra_train;i++)
  {
    y_scaled_ext[i] = sig(i)*(y_desired[NSamples-1] - y0);
    dy_ext[i] = sig(i)*dy_desired(NSamples-1);
    ddy_ext[i] = sig(i)*ddy_desired(NSamples-1);
    timed_ext[i] = i*dt ;
  }

  /* Printed in files for CHECK*/
  ofstream myFile1;

  std::ostringstream oss1;

  oss1 << "CHECK/sig" << id <<".log";
  myFile1.open((oss1.str()).c_str());
  for (int i = 0; i < NSamples+extra_train; i++)
    myFile1<<sig[i]<<endl;
  myFile1.close();

  oss1 << "CHECK/tsig" << id <<".log";
  myFile1.open((oss1.str()).c_str());
  for (int i = 0; i < NSamples+extra_train; i++)
    myFile1<<tsig[i]<<endl;
  myFile1.close();

  /* original signal */
  double fd_original_scaled[NSamples+extra_train];
  double fd_filtered[NSamples];

  vec fd_original_ext(3*NSamples+extra_train);
  vec fd_filtered_ext(3*NSamples+extra_train);

  /* find original scaled */
  for (int i = 0; i < NSamples+extra_train; i++)
  {
    fd_original_scaled[i] = ks*(y_scaled_ext[i]-y0) + y0; //basically is ext too
  }

  /* extent before filter */
  for (int i = 0; i < NSamples; i++)
  {
    fd_original_ext(i) = fd_original_scaled[0];
  }
  for (int i = 0; i < NSamples+extra_train; i++)
  {
    fd_original_ext( i + NSamples ) = fd_original_scaled[i] ;
  }
  for (int i = 2*NSamples+extra_train; i < 3*NSamples+extra_train; i++)
  {
    fd_original_ext(i) = fd_original_scaled[NSamples+extra_train-1];
  }

  double  tau = Tend, a = 10, b = a/4;

  /* Nyquist - fc */
  double fNyquist;
  fNyquist = fNyquistFunc(fd_original_scaled, NSamples+extra_train, 1/dt, Tend + extra_train*dt );

  BFs = ceil(Tend*fNyquist + extra_train*dt*fNyquist);

  std::cout << fNyquist << '\n';
  cout<<BFs<<endl;
  std::cout << Tend*fNyquist << '\n';
  std::cout << extra_train*dt*fNyquist << '\n';

  tNyq = 1/fNyquist;
//  Ns = ceil(NSamples/BFs);

  /* Lowpass Filter */
  // LowPassFilter lpf((1/(2*tNyq)), dt);
  // for(int i = 0; i < 3*NSamples+extra_train; i++)
  // {
  // 		fd_filtered_ext[i] = lpf.update(fd_original_ext[i]) ; //Update with 1.0 as input value.
  // }

  std::cout << "Filtering...";

  arma::vec filter_coeff = spl_::fir1(100 /*filter order*/, fNyquist*dt /*normalized cutoff Frequency*/);
  fd_filtered_ext = arma::conv(fd_original_ext, filter_coeff.t(), "same");
  std::cout << "[DONE]!\n";

  /* keep specific fd_filtered size of NSamples */
  for (int i = 0; i < NSamples+extra_train; i++)
  {
    fd_filtered[i] = fd_filtered_ext(i + NSamples) + y0;
  }

  /* A to return in once Reference y,dy,ddy */
  mat A(3,NSamples+extra_train); //3 y,dy,ddy not because of DIM
  for (int i = 0; i < NSamples+extra_train; i++)
  {
    A(0,i) = fd_filtered[i];
    A(1,i) = ks*kt*dy_ext[i];
    A(2,i) = ks*pow(kt,2)*ddy_ext[i];
  }

  /* Printed in files for CHECK*/
  ofstream myFile;

  std::ostringstream oss,oss2;

  oss << "CHECK/fd_ori" << id <<".log";
  myFile.open((oss.str()).c_str());
  for (int i = 0; i < NSamples+extra_train; i++)
    myFile<<fd_original_scaled[i]<<endl;
  myFile.close();

  oss2 << "CHECK/fd_filtered" << id <<".log";
  myFile.open((oss2.str()).c_str());
  for (int i = 0; i < NSamples+extra_train; i++)
    myFile<<fd_filtered[i]<<endl;
  myFile.close();

  // oss << "CHECK/drefD" << id <<".log";
  // myFile.open((oss.str()).c_str());
  // myFile<<A.row(1)<<endl;
  // myFile.close();
  //
  // oss << "CHECK/ddrefD" << id <<".log";
  // myFile.open((oss.str()).c_str());
  // myFile<<A.row(2)<<endl;
  // myFile.close();

  return A;
}

/* Calculate centers of kernels */
void GDMP::calculate_centers(vec timed)
{
  c.set_size(1,BFs);

  for (int idx = 0; idx<BFs; idx++)
  {
    c(idx) = (idx)*tNyq;
  }

  /* ext time for extra train*/
  vec timed_ext_train(NSamples+extra_train);
  for (int i = 0; i <NSamples; i++ )
    timed_ext_train(i) = timed(i);
  for (int i = 0; i < extra_train; i++ )
    timed_ext_train(i+NSamples) =  (i+1)*dt + timed(NSamples-1) ;

  /* Sincs == psi */
  psi.set_size(BFs , NSamples + extra_train);
  double x;
  for (int b = 0; b < BFs ; b++)
  {
    for (int i = 0; i <NSamples+ extra_train; i++ )
    {
      x = ((timed_ext_train(i) - c(b))/tNyq);
      if (x != 0)
        psi(b,i) = sin(M_PI*x)/(M_PI*x);
      else
        psi(b,i) = 1;
    }
  }

}

/* Weights */
void GDMP::generate_weights(mat fp_des)
{
  std::cout << "/* start generate weights */" << '\n';

  mat scaled_fp_d = fp_des.t() - y0*ones(NSamples+extra_train);

  w.set_size(BFs ,1);
  /* weights for BFs*/
  // !! DERIVATE WITH 0 ?
  if ( g == y0 )
  {
    for (int idx = 0; idx<BFs; idx++)
    {
        w(idx) = 1;
    }
  }
  else
  {
    for (int idx = 0; idx<BFs; idx++)
    {
        w(idx) = scaled_fp_d(round(c(idx)/dt))/(g - y0);
    }
  }

}


vec GDMP::init_solution_dt(double goal, CanonicalStructure cs, double main_time, int extra_samples, double t_now, int i)
{
  /* Set size = NSamples + extra samples
      extra samples oq from extra time to run
  */
  y.set_size(1,NSamples+extra_samples);
  dy.set_size(1,NSamples+extra_samples);
  ddy.set_size(1,NSamples+extra_samples);

  az = 10;
  bz = az/4;

  K = az*bz/pow(cs.taf,2);
  D = (az + cs.dtaf)/cs.taf;

  k = goal - y0;

  /* Init fp*/
  double s = 0.0 ;
  for (int l = 0; l < BFs ; l++)
  {
    s = s + w(l)*psi(l,i);
  }

  /* Declare double mat vec fp d dd */
  double fp =  k*s + y0;
  double dfp = 0;
  double ddfp = 0;

  y0d_hat = fp;

  /* needed to get gd_hat */
  s = 0.0 ;
  for (int l = 0; l < BFs ; l++)
  {
    s = s + w(l)*psi(l,NSamples-1);
  }

  gd_hat = k*s + y0;

  // // !! DERIVATE WITH 0 ?
  // if (gd_hat == y0d_hat)
  // {
  // }
  // else
  // {
  //   ks = (goal - y0) / (gd_hat - y0d_hat);
  // }
  ks = 1;

  /* Ref compute*/
  double y_ref  = ks*(fp - y0d_hat) + y0;
  double dy_ref  = ks*dfp ;
  double ddy_ref  = ks*ddfp ;

  /* first time loop i = 0*/

  // y dy ddy: only them are stored
  y(i) = y0;
  dy(i) = 0; //dy0;
  double fs = ddy_ref + D*dy_ref + K*(y_ref-goal);
  ddy(i) = K*(goal-y(i)) - D*dy(i) + fs;

  vec info_back(3);
  info_back(0)  = fp;
  info_back(1)  = dfp;
  info_back(2)  = ddfp;

  return info_back;
}

 vec GDMP::run_solution_dt(double t_now, double goal, double ts, int i, double f_prev, double df_prev, double ddf_prev )
 {
     double fp  ;
     double dfp  ;
     double ddfp ;

     /* Compute fp */
     if (i > (NSamples-1))
     {
         fp = f_prev  ; //same as prev or (NSamples-1)
         dfp = df_prev ;
         ddfp = df_prev;
     }
     else
     {
         double s = 0.0 ;
         for (int l = 0; l < BFs ; l++)
         {
           s = s + w(l)*psi(l,i);
         }
         fp = k*s + y0;
         dfp =  (fp - f_prev)/ts;
         ddfp = (dfp - df_prev)/ts;
     }

     /* Compute refs */
     double y_ref  = ks*(fp - y0d_hat) + y0;
     double dy_ref  = ks*dfp ;
     double ddy_ref  = ks*ddfp ;

     /* Euler Solution dt*/
     dy(i) = ddy(i-1)*ts + dy(i-1);
     y(i) = dy(i)*ts + y(i-1);

     /* dynamical system */
     double fs = ddy_ref + D*dy_ref + K*(y_ref-goal);
     ddy(i) = K*(goal-y(i)) - D*dy(i) + fs;

     /* keep it for next loop dt*/
     vec info_back(3);
     info_back(0)  = fp;
     info_back(1)  = dfp;
     info_back(2)  = ddfp;

     return info_back;
 }


vec GDMP::init_Rsolution_dt(double goal, CanonicalStructure cs, double main_time, int extra_samples, double t_now, int i)
{
  /* Set size = NSamples + extra samples
      extra samples oq from extra time to run
  */
  Ry.set_size(1,NSamples+extra_samples);
  Rdy.set_size(1,NSamples+extra_samples);
  Rddy.set_size(1,NSamples+extra_samples);



  /* Swap gd,y0 */
  double Ry0 = goal;
  double Rgoal = y0;

  /* Swap gd,y0 hats */
  Ry0d_hat = Ry0; //gd_hat;
  Rgd_hat = Rgoal; //y0d_hat;

  /* init Rfp */
  double Rfp = gd_hat;
  double Rdfp = 0;
  double Rddfp = 0;

  /* Ref compute */
  double Ry_ref  = ks*(Rfp - Ry0d_hat) + Ry0;
  double Rdy_ref  = ks*Rdfp ;
  double Rddy_ref  = ks*Rddfp ;

  /* Init R solutions dt */
  Ry(i) = goal;
  Rdy(i) = 0; //Ry0;
  double Rfs = Rddy_ref + D*Rdy_ref + K*(Ry_ref-Rgoal);
  Rddy(i) = K*(Rgoal-Ry(i)) - D*Rdy(i) + Rfs;

/* keep it for the nect loop dt */
 vec info_back(3);
 info_back(0)  = Rfp;
 info_back(1)  = Rdfp;
 info_back(2)  = Rddfp;

 return info_back;
}

vec GDMP::run_Rsolution_dt(double t_now, double goal, double ts, int i, double f_prev, double df_prev, double ddf_prev )
{
    double Rfp  ;
    double Rdfp  ;
    double Rddfp ;
    double Ry0 = goal;
    double Rgoal = y0;

    /* Compute Rfp */
    if (i > (NSamples-1))
    {
        Rfp = f_prev  ; //same as prev or (NSamples-1)
        Rdfp = df_prev ;
        Rddfp = df_prev;
    }
    else
    {
        double s = 0.0 ;
        for (int l = 0; l < BFs ; l++)
        {
          s = s + w(l)*psi(l,NSamples - i );
        }

        Rfp = k*s + y0;
        Rdfp =  (Rfp - f_prev)/ts;
        Rddfp = (Rdfp - df_prev)/ts;
    }

    /* Compute R refs */
    double Ry_ref  = ks*(Rfp - Ry0d_hat) + Ry0;
    double Rdy_ref  = ks*Rdfp ;
    double Rddy_ref  = ks*Rddfp ;

    /* Euler Solution dt*/
    Rdy(i) = Rddy(i-1)*ts + Rdy(i-1);
    Ry(i) = Rdy(i)*ts + Ry(i-1);

    /* Dynamical system dt*/
    double Rfs = Rddy_ref + D*Rdy_ref + K*(Ry_ref-Rgoal);
    Rddy(i) = K*(Rgoal-Ry(i)) - D*Rdy(i) + Rfs;

    /* keep it for the nect loop dt */
    vec info_back(3);
    info_back(0)  = Rfp;
    info_back(1)  = Rdfp;
    info_back(2)  = Rddfp;

    return info_back;
}

/* fNyquist Function*/
double GDMP::fNyquistFunc(double *original, int size, double fs, double T)
{
  int L;
  L = (size % 2) == 0 ? size : size+1;

  vec signal_in(size);
  // cx_vec signal_xf(size);
  //
  // cx_mat XF_in(1,int(l1/2+1));
  // vec A1(size);
  //
  double f[int(L/2)];
  for ( int i = 0; i < (L/2); i++ )
      f[i] = fs*i/L;
  // complex<double> epi;
  // double EdXF = 0;
  // double  img_EdXF = 0;
  for (int i = 0; i<size ; i++)
    signal_in(i) = original[i];

  cx_vec P2(size);
  P2 = fft(signal_in) ;

  cx_vec Fd_original(int(L/2));
  for ( int i = 0; i < (L/2); i++ )
      Fd_original(i) = P2(i);

  double EdD = 0;
  for ( int i = 0; i < (L/2); i++)
  {
    if (f[i] < 40)
      EdD += abs(pow(Fd_original(i),2));
  }

  double h = 0.001;
  double EdDesired = (1-h)*EdD;
  double energy_inter = 0;

  int i = -1;
  while( energy_inter < EdDesired)
  {
    i++;
    energy_inter = energy_inter + abs(pow(Fd_original(i),2));
  }
  // cout<<i<<endl;
  double fNyq ;

  if ( i < 0)
    fNyq = 2*f[1];
  else
    fNyq = 2*f[i];

  return fNyq;
}
