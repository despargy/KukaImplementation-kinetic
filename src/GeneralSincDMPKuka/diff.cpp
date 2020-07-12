#include <iostream>
#include <armadillo>

using namespace std;
using namespace arma;

double* diff(double *y, int len, double dt)
{
  double *dy;
  dy = new double[len];

  dy[0] = y[0];
  for (int i = 1; i<len; i++)
  {
    dy[i] = (y[i] - y[i-1])/dt;
  }
  return dy;
}

mat diffMat()
{
  mat dy() ; 
  return dy;
}
