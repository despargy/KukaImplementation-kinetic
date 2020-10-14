#include <fstream>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <armadillo>
using namespace arma;
using namespace std;

mat readQ()
{

  /* read */

    // std::ifstream ifile("Collector/posmat.txt", std::ios::in);
    std::ifstream ifile("Collector/posCollected_cut.txt", std::ios::in);

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

    // //verify that the scores were stored correctly:
    // for (int i = 0; i < int(scores.size()/DIM); ++i) {
    //     for (int e = 0; e<DIM; e++)
    //     {
    //       // std::cout << scores[i+e] << "\t";
    //       std::cout << m(e,i) << std::endl ;
    //     }
    // }

    return m;
}
