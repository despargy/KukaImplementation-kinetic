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
  myFile.open("Collector/pos_got.log");
  for (int i = 0;i<10;i++)
    myFile<<v;
  myFile.close();


  //// get
    FILE *fp;
    char ch=' ';
    int cnt=0;
   	fp=fopen("Collector/pos_got.log","r");
    if (fp==NULL) exit(2);
    vec vv(1);
    while((ch=fgetc(fp))!=EOF)
    {
	     vv(1) = putchar(ch);

    }

    fclose(fp);
    printf("\n b meros\n");
    //------------------------------------------
    FILE *fp1;
    int ar0,ar1,ar2,ar3,ar4,ar5,ar6,k;
     fp1=fopen("Collector/pos_got.log","r");
     if (fp1==NULL)
     {
        puts("Provlima sto anoigma toy arxeioy");
        exit(2);
     }

      while (k=fscanf(fp1,"%d %d %d %d %d %d %d", &ar0, &ar1, &ar2, &ar3, &ar4, &ar5, &ar6)>=1)
     {

	   //  a++;
	   //  printf("%d",a);

         printf("%d %d %d %d %d %d %d \n",ar0,ar1,ar2, ar3,ar4,ar5,ar6);
         fgetc(fp1);
     }
     fclose(fp1);

    return 0;
}
