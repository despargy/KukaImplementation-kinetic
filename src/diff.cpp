#include <iostream>
#include <armadillo>

using namespace std;
using namespace arma;

double* diff_ts(double *y, int len, double dt)
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

mat diffMat( int len, int DIM, double dt, mat y)
{
  mat dy(DIM, len) ;
  for (int d = 0; d<DIM; d++)
  {
    for (int i = 1; i<len; i++)
    {
      dy(d,i) = (y(d,i) - y(d,i-1))/dt;
    }
    dy(d,0) = dy(d,1);

  }
  return dy;

}
