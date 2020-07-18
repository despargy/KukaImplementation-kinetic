#include <stdio.h>
#include <stdlib.h>
#include <armadillo>
#include <iostream>


using namespace arma;
using namespace std;

int main(void)
{
///read
  ofstream myFile;
  vec v(7);
  for (int i = 0;i<7;i++)
    v(i) = i;
   // v = {1, 2, 3, 4, 5, 6, 7};
  myFile.open("Collector/todelete.log");
  for (int i = 0;i<20;i++)
    myFile<<v;
  myFile.close();


    /* read */

      std::ifstream ifile("Collector/todelete.log", std::ios::in);
      // std::ifstream ifile("Collector/posCollected.txt", std::ios::in);

      std::vector<double> scores;

      //check to see that the file was opened correctly:
      if (!ifile.is_open()) {
          std::cerr << "There was a problem opening the input file!\n";
          exit(1);//exit or do additional error checking
      }

      int DIM = 7;
      double num = 0.0;
      int i = 0;

      //keep storing values from the text file so long as data exists:
      while (ifile >> num) {
          scores.push_back(num);
      }

      mat m(DIM,int(scores.size()/DIM));
      int t=-1;

      for (int i = 0; i < int(scores.size()/DIM); ++i)
      {
        for (int e = 0; e<DIM; e++)
        {
          t++;
          m(e,i) = scores[t] ;
        }
      }
cout<<"start out m"<<endl;
      //verify that the scores were stored correctly:
      for (int i = 0; i < int(scores.size()/DIM); ++i) {
          for (int e = 0; e<DIM; e++)
          {
            // std::cout << scores[i+e] << "\t";
            std::cout << m(e,i) << "\t" ;
          }
          cout<<endl;
      }

  //// get
    // FILE *fp;
    // char ch=' ';
    // int cnt=0;
   	// fp=fopen("Collector/todelete.log","r");
    // if (fp==NULL) exit(2);
    // vec vv(1);
    // while((ch=fgetc(fp))!=EOF)
    // {
	  //    vv(1) = putchar(ch);
    //
    // }
    //
    // fclose(fp);
    // printf("\n b meros\n");
    // //------------------------------------------
    // FILE *fp1;
    // int ar0,ar1,ar2,ar3,ar4,ar5,ar6,k;
    //  fp1=fopen("Collector/todelete.log","r");
    //  if (fp1==NULL)
    //  {
    //     puts("Provlima sto anoigma toy arxeioy");
    //     exit(2);
    //  }
    //
    //   while (k=fscanf(fp1,"%d %d %d %d %d %d %d", &ar0, &ar1, &ar2, &ar3, &ar4, &ar5, &ar6)>=1)
    //  {
    //
	  //  //  a++;
	  //  //  printf("%d",a);
    //
    //      printf("%d %d %d %d %d %d %d \n",ar0,ar1,ar2, ar3,ar4,ar5,ar6);
    //      fgetc(fp1);
    //  }
    //  fclose(fp1);

    return 0;
}
