#include <armadillo>
using namespace arma;

vec dataFormer(vec x, double dt)
{
  vec dx(size(x));

  vec p1 = diff(x)/dt;
  dx(0) = p1(0);
  for (int i = 1; i< x.n_elem; i++)
  {
    dx(i) = p1(i-1);
  }
  // std::cout << "p1 = " <<p1<< '\n';
  // std::cout << "dx = " <<dx<< '\n';
  return dx;
}

mat dataFormer(mat x, double dt)
{
  mat dx(size(x));

  mat p1 = diff(x)/dt;
  dx(0) = p1(0);
  for (int i = 1; i< x.n_elem; i++)
  {
    dx(i) = p1(i-1);
  }
  // std::cout << "p1 = " <<p1<< '\n';
  // std::cout << "dx = " <<dx<< '\n';
  return dx;
}
