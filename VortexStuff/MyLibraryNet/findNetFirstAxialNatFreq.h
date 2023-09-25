// Created by Eleonora Botta, September 10, 2016
// Modified by Eleonora Botta, March 10, 2017: pass mass of knots 

#ifndef FINDNETFIRSTAXIALNATFREQ_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define FINDNETFIRSTAXIALNATFREQ_H

// Include all libraries needed by cpp files in this header
#include <iostream>

// third party libraries
//#include "Eigen\Eigen" // for computing eigenvalues

// define headers of cpp functions
double findNetFirstAxialNatFreq(int NetSize, int CMsPresent, float massHalfThread, float massHalfCornerThread, float massKnot, float threadStiffness );

#endif