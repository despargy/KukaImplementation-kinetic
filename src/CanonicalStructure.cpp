// CANONICALSTRUCTURE class for General DMPs
//   Let x spread in time
//   D.E Argiropoulos - General DMP - June 2020

#include <iostream>
#include <string>
#include <armadillo>

using namespace std;
using namespace arma;

class CanonicalStructure
{
public:
    double ax , x0, Rx0;
    vec x, dx;
    vec Fx, fx;
    vec Rdx, Rx;
    double t0, tf, T;
    double taf , dtaf ;

    CanonicalStructure(int len);
    ~CanonicalStructure();
    void generateFx(std::string kind, vec timed);
    double getFx(double t);
    void generateRfx(string kind, vec timed);

};

CanonicalStructure::CanonicalStructure(int len)
{

  x.set_size(len);
  dx.set_size(len);
  Fx.set_size(len);
  fx.set_size(len);
  Rx.set_size(len);
  Rdx.set_size(len);

  std::cout << "/* Constructor of CanonicalStructure */" << '\n';
}
CanonicalStructure::~CanonicalStructure()
{
}

void CanonicalStructure::generateFx(string kind, vec timed)
{
  int len = timed.n_rows;
  x0 = 0.01;
  t0 = 0;
  ax = 1/2;
  T = timed[len-1]; // starts from 0sec - (len-1)*ts
  tf = T + t0;
  taf = 1;
  dtaf = 0;
  if ( kind == "discrete")
  {
    //CS for discrete
    for (int i = 0; i<len; i++)
    {
      Fx[i] =  ax*timed[i];
      fx[i] = ax;
      x[i] = x0 + Fx[i]/taf ;
      dx[i] = fx[i]/taf;
    }
  }
}

double CanonicalStructure::getFx(double t)
{
  return ax*t;
}

void CanonicalStructure::generateRfx(string kind, vec timed)
{
  int len = timed.n_rows;
  Rx0 = x[len-1] ;
  if ( kind == "discrete")
  {
    for (int i = 0;i<len;i++)
    {
      Rx[i] = Rx0 - Fx[i]/taf ;
      Rdx[i] = -fx[i]/taf;
    }

  }
}
