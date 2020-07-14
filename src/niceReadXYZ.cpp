#include <stdio.h>
#include <stdlib.h>
#include <armadillo>
#include <iostream>
#include <string>

mat niceReadXYZ(string arxeio)
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
  int DIM = 3;
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
     int ar0,ar1,ar2,k;

     fp1=fopen(arxeio.c_str(),"r");
     if (fp1==NULL)
     {
        puts("Error opening file");
        exit(2);
     }

      while (k=fscanf(fp1,"%d %d %d", &ar0, &ar1, &ar2)>=1)
     {

	      i++;
        m(0,i) = ar0;
        m(1,i) = ar1;
        m(2,i) = ar2;

         // printf("%d %d \n",ar1,ar2);

     }
     fclose(fp1);

    return m;
}
