//! \file filters.h
//!
//! \brief
//! This code implements FIR and IIR filters
//!
//! \date September 2017
//! \version 1.0.0
//! \authors Dimitrios Papageorgiou, email: dimpapag@iti.gr

//#include <arl_dmp/filters.h>


class Filter
{
public:
  int Ncoeffs;

  Filter();
  Filter(int Ncoeffs_in);

  arma::mat filterOffline(arma::mat x_in);
  double filterOnline(arma::mat x_in, int i);
  void setNcoeffs(int Ncoeffs_in);
};

/**
 * @brief Filter::Filter, empty constructor
 */
Filter::Filter()
{
    std::cout << "[Filter::Filter] Filter  is created, but not initialized." << std::endl;
}


/**
 * @brief Filter::Filter, zero phase MA filter constructor
 * @param coeffs_in, the filter coefficients (vector)
 * @param phase_in, the phase of the output in samples
 */
Filter::Filter(int Ncoeffs_in)
{
    std::cout << "[Filter::Filter] FIR filter is created." << std::endl;
    std::cout << "[Filter::Filter] Initializing parameters" << std::endl;
    Ncoeffs = Ncoeffs_in;
}

void Filter::setNcoeffs(int Ncoeffs_in)
{
    Ncoeffs = Ncoeffs_in;
}

/**
 * @brief Filter::filterOffline, filters a time series
 * @param x_in, the input signal
 * @return the filtered signal
 */
arma::mat Filter::filterOffline(arma::mat x_in){

  int N;
  int dim;

  N = x_in.n_cols;
  dim = x_in.n_rows;

  arma::mat x_out = arma::zeros<arma::mat>(dim,N);
  int start_index;
  int end_index;


  for(int i = 0; i< N ; i++)
  {
	     start_index = std::max( i - Ncoeffs/2 , 0);
	     end_index = std::min( i + (Ncoeffs - Ncoeffs/2), N-1);

	     x_out.col(i) = arma::mean( x_in.cols(start_index , end_index ), 1 ) * ((double)(end_index - start_index))/ ((double)Ncoeffs);
  }

  return x_out;

}

double Filter::filterOnline(arma::mat x_in, int i){
  if (i < Ncoeffs)
  {
    for (int l = i + 1; l < Ncoeffs; l++)
      x_in(0,l) = x_in(0,0);
  }
  int end_index = std::max( i, Ncoeffs-1) ;
  int start_index = std::max(i - Ncoeffs + 1, 0); //
  arma::mat a(1,1);
  a = arma::mean( x_in.cols(start_index , end_index ), 1 );// * ((double)(end_index - start_index))/ ((double)Ncoeffs);
  return a(0,0);

}
