#include <iostream>
#include <string>
#include <armadillo>

using namespace std;
using namespace arma;

mat loadData( string arxeio)
{
  int DIM = 1;
  ifstream myFile;

  // choose data
  try
  {
    myFile.open(arxeio.c_str());
  }
  catch(const std::exception& e)
  {
    cout<< "Failed to open file data "<<arxeio<< endl;
  }


  //get size
  double num = 0.0;
  int lenght =0;
  while (myFile >> num)
  {
    lenght++;
  }
  myFile.close();

  // fix size
  mat m(DIM,lenght);

  // open again to get data
  myFile.open(arxeio.c_str());
  num = 0.0;
  int l = 0;

  // for (int d = 0 ; d++; d < DIM)
  // {
    while (myFile >> num)
    {
      m(0,l) = num;
      l++;
    }
  // }

  myFile.close();
  return m;
}
