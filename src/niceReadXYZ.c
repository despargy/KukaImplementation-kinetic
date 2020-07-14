#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    mat var();
    int i = -1;

    FILE *fp1;
     int i,ar0,ar1,ar2,k;

     fp1=fopen("/Collector/txy.txt","r");
     if (fp1==NULL)
     {
        puts("Error opening file");
        exit(2);
     }

      while (k=fscanf(fp1,"%d %d %d", &ar0, &ar1, &ar2)>=1)
     {

	      i++;
        var(0,i) = ar0;
        var(1,i) = ar1;
        var(2,i) = ar2;

         // printf("%d %d \n",ar1,ar2);

     }
     fclose(fp1);

    return 0;
}
