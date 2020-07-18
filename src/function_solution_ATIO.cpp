void GDMP::run_solution(double goal, CanonicalStructure cs, double ts, double main_time, double extra_time, int extra_samples)
{
   /* Init mat sizes*/
   mat fp(1,NSamples+extra_samples);
   mat dfp(1,NSamples+extra_samples);
   mat ddfp(1,NSamples+extra_samples);
   mat fs(1,NSamples+extra_samples);
   y.set_size(1,NSamples+extra_samples); dy.set_size(1,NSamples+extra_samples); ddy.set_size(1,NSamples+extra_samples);
   mat y_ref(1,NSamples+extra_samples);
   mat dy_ref(1,NSamples+extra_samples);
   mat ddy_ref(1,NSamples+extra_samples);

  double az = 10;
  double bz = az/4;

  double K = az*bz/pow(cs.taf,2);
  double D = (az + cs.dtaf)/cs.taf;

  double k = goal - y0;
  double t = 0;
  double gama = 1 - exp(-1000*t);

  int i = 0;

  /* Init fp*/
  double s = 0.0 ;
  for (int l = 0; l < BFs + extra_train; l++)
  {
    s = s + w(l)*psi(l,i);
  }

  fp(i) =  gama*k*s + y0;
  dfp(i) = 0;
  ddfp(i) = 0;
  y0d_hat = fp(i);

  s = 0.0 ;

  for (int l = 0; l < BFs + extra_train; l++)
  {
    s = s + w(l)*psi(l,NSamples-1);
  }

  gd_hat = (1 - exp(-1000*main_time))*k*s + y0;
  double ks = (goal - y0) / (gd_hat - y0d_hat);

  /* Ref compute*/
  y_ref(i)  = ks*(fp(i) - y0d_hat) + y0;
  dy_ref(i)  = ks*dfp(i) ;
  ddy_ref(i)  = ks*ddfp(i) ;

  /* first time loop i = 0*/
  y(i) = y0;
  dy(i) = dy0;
  fs(i) = ddy_ref(i) + D*dy_ref(i) + K*(y_ref(i)-goal);
  ddy(i) = K*(goal-y(i)) - D*dy(i) + fs(i);

  /* fix sigmoid*/
  mat tsig = linspace(-main_time-10,extra_time,NSamples+extra_samples);
  mat sig = 1-1/(1+exp(-tsig));

  // /* ROS publisher*/
  // ros::NodeHandle n;
  // ros::Publisher solution_pub = n.advertise<std_msgs::String>("chatter", 1000);

  for( double t = ts ; t < (main_time + extra_time + ts); t = t + ts)
  {

    i++;

    // fp_compute
    if (i > (NSamples-1))
    {
        fp(i) = fp(NSamples-1) ;
        dfp(i) = dfp(NSamples-1) ;
        ddfp(i) = dfp(NSamples-1);
    }
    else
    {
        s = 0.0 ;
        for (int l = 0; l < BFs + extra_train; l++)
        {
          s = s + w(l)*psi(l,i);
        }
        gama = 1 - exp(-1000*t);
        fp(i) = gama*k*s + y0;
        dfp(i) =  (fp(i) - fp(i-1))/ts;
        ddfp(i) = (dfp(i) - dfp(i-1))/ts;
    }

    //y ref_compute
    y_ref(i)  = ks*(fp(i) - y0d_hat) + y0;
    dy_ref(i)  = ks*dfp(i) ;
    ddy_ref(i)  = ks*ddfp(i) ;

    // Euler Solution
    dy(i) = ddy(i-1)*ts + dy(i-1);
    y(i) = dy(i)*ts + y(i-1);

    // dynamical system
    fs(i) = ddy_ref(i) + D*dy_ref(i) + K*(y_ref(i)-goal);
    ddy(i) = K*(goal-y(i)) - D*dy(i) + sig(i)*fs(i);

    // /* ros publisher*/
    // if (ros::ok())
    // {
    //   std_msgs::String msg;
    //   std::stringstream ss;
    //   ss << "hello world " << i;
    //   msg.data = ss.str();
    //   ROS_INFO("%s", msg.data.c_str());
    //   solution_pub.publish(msg);
    //   ros::spinOnce();
    // }
    // else
    // {}
  }
            ofstream myFile;
            myFile.open("CHECK/fs.log");
            myFile<<fs<<endl;
            myFile.close();

            myFile.open("CHECK/y.log");
            myFile<<y<<endl;
            myFile.close();

            myFile.open("CHECK/dy.log");
            myFile<<dy<<endl;
            myFile.close();

            myFile.open("CHECK/ddy.log");
            myFile<<ddy<<endl;
            myFile.close();

}
