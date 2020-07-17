#include <stdio.h>
#include <stdlib.h>
#include <armadillo>
#include <iostream>
#include <string>

mat niceRead(string arxeio)
{
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
  int DIM = 7;
  double num = 0.0;
  int lenght =0;
  while (myFile >> num)
  {
    lenght++;
  }
  myFile.close();
  lenght = int(lenght/DIM);
  // fix size
  mat m(DIM,lenght);

  // open again to get data

    int i = -1;

    FILE *fp1;
     int ar0,ar1,ar2,ar3,ar4,ar5,ar6,k;

     fp1=fopen(arxeio.c_str(),"r");
     if (fp1==NULL)
     {
        puts("Error opening file");
        exit(2);
     }

      while (k=fscanf(fp1,"%d %d %d %d %d %d %d", &ar0, &ar1, &ar2, &ar3, &ar4, &ar5, &ar6)>=1)
     {

	      i++;
        m(0,i) = ar0;
        m(1,i) = ar1;
        m(2,i) = ar2;
        m(3,i) = ar3;
        m(4,i) = ar4;
        m(5,i) = ar5;
        m(6,i) = ar6;

         printf("%d %d %d %d %d %d %d", &ar0, &ar1, &ar2, &ar3, &ar4, &ar5, &ar6));

     }
     fclose(fp1);

    return m;
}
