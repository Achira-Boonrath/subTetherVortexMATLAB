// Created by Eleonora Botta, September 16, 2016

#ifndef ADDTARGET_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ADDTARGET_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/readParameter.h"

#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxSim/VxApplication.h>
#include <VxMath/Transformation.h>
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>

#include <iostream>
#include <fstream>
#include <cmath>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
VxPart* addTarget(Vx::VxSmartPtr<VxSim::VxApplication> application, std::string pathData);

#endif