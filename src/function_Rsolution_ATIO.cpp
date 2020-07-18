void GDMP::run_reverse_solution(double goal, CanonicalStructure cs, double ts, double main_time, double extra_time, int extra_samples)
{

    /* Init mat sizes*/
    mat Rfp(1,NSamples+extra_samples);
    mat Rdfp(1,NSamples+extra_samples);
    mat Rddfp(1,NSamples+extra_samples);
    mat Rfs(1,NSamples+extra_samples);
    Ry.set_size(1,NSamples+extra_samples); Rdy.set_size(1,NSamples+extra_samples); Rddy.set_size(1,NSamples+extra_samples);
    mat Ry_ref(1,NSamples+extra_samples);
    mat Rdy_ref(1,NSamples+extra_samples);
    mat Rddy_ref(1,NSamples+extra_samples);

    double az = 10;
    double bz = az/4;

    double K = az*bz/pow(cs.taf,2);
    double D = (az + cs.dtaf)/cs.taf;

    double Ry0d_hat = gd_hat;
    double Rgd_hat = y0d_hat;

    double Ry0 = goal;
    double Rgoal = y0;

    double ks = (goal - y0) / (gd_hat - y0d_hat);

    double k = goal - y0;
    double t = 0;
    double gama = 1 - exp(-1000*t);

    int i = 0;

    /* Init fp*/
    double s = 0.0 ;

    Rfp(0) = ks*gd_hat + y0;
    Rdfp(0) = 0;
    Rddfp(0) = 0;

    /* Ref compute*/
    Ry_ref(0)  = ks*(Rfp(0) - Ry0d_hat) + Ry0;
    Rdy_ref(0)  = ks*Rdfp(0) ;
    Rddy_ref(0)  = ks*Rddfp(0) ;

    Ry(i) = Ry0;
    Rdy(i) = Ry0;
    Rfs(i) = Rddy_ref(i) + D*Rdy_ref(i) + K*(Ry_ref(i)-y0);
    Rddy(i) = K*(y0-Ry(i)) - D*Rdy(i) + Rfs(i);

    mat tsig = linspace(-main_time-10,extra_time,NSamples+extra_samples);
    mat sig = 1-1/(1+exp(-tsig));

  for(double t = ts ; t < (main_time + extra_time + ts); t = t + ts)
  {
    i++;

    // fp_reverse_compute
    if (i > (NSamples-1))
    {
        Rfp(i) = Rfp(NSamples-1) ;
        Rdfp(i) = Rdfp(NSamples-1) ;
        Rddfp(i) = Rdfp(NSamples-1) ;
    }
    else
    {
        gama = 1 - exp(-1000*t);

        s = 0.0 ;
        for (int l = 0; l < BFs + extra_train; l++)
        {
          s = s + w(l)*psi(l,NSamples - i - 1);
        }
        Rfp(i) = k*s + y0;
        Rdfp(i) =  (Rfp(i) - Rfp(i-1))/ts;
        Rddfp(i) = (Rdfp(i) - Rdfp(i-1))/ts;
    }

    //y ref_compute
    Ry_ref(i)  = ks*(Rfp(i) - Ry0d_hat) + Ry0;
    Rdy_ref(i)  = ks*Rdfp(i) ;
    Rddy_ref(i)  = ks*Rddfp(i) ;

    // Euler Solution
    Rdy(i) = Rddy(i-1)*ts + Rdy(i-1);
    Ry(i) = Rdy(i)*ts + Ry(i-1);

    // dynamical system
    Rfs(i) = Rddy_ref(i) + D*Rdy_ref(i) + K*(Ry_ref(i)-Rgoal);
    Rddy(i) = K*(Rgoal-Ry(i)) - D*Rdy(i) + sig(i)*Rfs(i);
  }
            ofstream myFile;
               myFile.open("CHECK/Ry.log");
               myFile<<Ry<<endl;
               myFile.close();

               myFile.open("CHECK/Rdy.log");
               myFile<<Rdy<<endl;
               myFile.close();

               myFile.open("CHECK/Rddy_ref.log");
               myFile<<Rddy<<endl;
               myFile.close();
}
