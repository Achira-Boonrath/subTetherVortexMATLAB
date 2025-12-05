// Created by Eleonora Botta, March 28, 2019

// Read the interpolated residual gravity files for experiment on capture of envisat mockup

#ifndef GETEXPGRAVITY_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define GETEXPGRAVITY_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <sstream>
#include <fstream>

//#define MAXTIMESTEPSVALIDATION 20000


//using namespace Vx;
//using namespace VxDynamics;
//using namespace VxSim;

// define headers of cpp functions
void getParabolicExperimentResidualGravity(std::string pathData, float tstep, float myArray[][3]);

void getParabolicExperimentRotVel(std::string pathData, float tstep, float myArray[][3]);



#endif