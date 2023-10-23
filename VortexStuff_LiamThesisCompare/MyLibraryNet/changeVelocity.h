// Created by Eleonora Botta, September 29, 2016


#ifndef CHANGEWINCHANGULARVELOCITY_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define CHANGEWINCHANGULARVELOCITY_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/readParameter.h"

#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Hinge.h>

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
float changeVelocity(VxSmartInterface<Hinge>* hingeForWinchPtr, float currentTime, float functionTimes[], std::string functionControls[], float functionVelocities[], float functionAccelerations[], int sizeFunctionTimes, float winchRadius);

#endif