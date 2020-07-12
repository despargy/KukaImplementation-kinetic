#include "mat.h"
#include <iostream>
#include <vector>

void matread(const char *file, std::vector<double>& v)
{
    // open MAT-file
    MATFile *pmat = matOpen(file, "r");
    if (pmat == NULL) return;

    // extract the specified variable
    mxArray *arr = matGetVariable(pmat, "LocalDouble");
    if (arr != NULL && mxIsDouble(arr) && !mxIsEmpty(arr)) {
        // copy data
        mwSize num = mxGetNumberOfElements(arr);
        double *pr = mxGetPr(arr);
        if (pr != NULL) {
            v.reserve(num); //is faster than resize :-)
            v.assign(pr, pr+num);
        }
    }

    // cleanup
    mxDestroyArray(arr);
    matClose(pmat);
}

// int main()
// {
//     std::vector<double> v;
//     matread("data.mat", v);
//     for (size_t i=0; i<v.size(); ++i)
//         std::cout << v[i] << std::endl;
//     return 0;
// }
