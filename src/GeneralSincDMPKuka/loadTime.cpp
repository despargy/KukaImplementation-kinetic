#include <iostream>
#include <string>
#include <armadillo>

using namespace std;
using namespace arma;

mat loadTime(int lenght_)
{
  ifstream myFile;

  // fix size
  vec t(lenght_);

  // open again to get data
  myFile.open("data/DataTimed.txt");
  double num = 0.0;
  int l = 0;

  while (myFile >> num)
  {
    t(l) = num;
    l++;
  }

  myFile.close();
  return t;
}
