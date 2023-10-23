

#include "MyLibraryNet/matrixMult.h"

#include<iostream>
#include<math.h>
#include<vector>

using namespace std;

//std::vector<float> matrixMult(float a[], std::vector<float> b)



std::vector<double> matrixMult(double a[3][6], vector<double>& b)
{


    const int rows = 3;
    const int cols = 6;
    //int rows = std::extent<decltype(array), 0>::value;
    vector<double> c;
  

    for (int i = 0; i < rows; i++)
    {
        c.push_back(0);
    }

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            c[i] += (a[i][j] * b[j]);
        }
    }

    return c;
}