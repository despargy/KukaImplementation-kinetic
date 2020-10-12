#define SUBBLOCK SIZE 22 // for method 0, and 12 for methods 1 and 2
void fir(double * const x, const int Nx,
const double * const h, const int Nh,
double* r)
{
/*
x:
Nx:
h:
Nh:
input data
length of input data
filter coefficients
length of the filter
21r:
the result after filtering
*/
}
int main()
{
  const double h[] = {. . .}; // the filter coefficients
  const int FILTER LEN = sizeof(h)/sizeof(double);
  const int HALF FILTER LEN = (FILTER LEN-1)/2;
  const int BLOCK SIZE = SUBBLOCK SIZE + (FILTER LEN-1);
  double x[BLOCK SIZE]; // input data
  double r[BLOCK SIZE]; // result
  int i = HALF FILTER LEN;
  memset(x, 0, sizeof(double)*BLOCK SIZE); // init x
  while(cin >> x[i]){

    if(++i >= BLOCK SIZE){
      // perform FIR filtering
      fir(x, BLOCK SIZE, h, FILTER LEN, r);
      // make result non-causal by shifting result to the left
      // so that only array values from 0 to SUBBLOCK SIZE-1
      // are valid
      memmove(r, r+FILTER LEN-1, SUBBLOCK SIZE*sizeof(double));
      // show the filtered result
      for(int j=0; j < SUBBLOCK SIZE; j++){
        cout << r[j] << "\n";
      }
      // copy the old x values at the end to the beginning of x
      memmove(x, x+(BLOCK SIZE-FILTER LEN+1),
      (FILTER LEN-1)*sizeof(double));
      // reset counter
      i=FILTER LEN-1;

    }
  }
}
