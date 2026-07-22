// Created by Eleonora Botta, September 28, 2016

#ifndef SAVECONTACTINFORMATION_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define SAVECONTACTINFORMATION_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include "Vx/VxDynamicsResponseInput.h"
#include <Vx/VxContactMaterial.h>
#include <Vx/VxUniverse.h>
#include <VxDynamics/Part.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

void saveContactInformation(VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], float currentTime, int NetSize, int CMsPresent, FILE* outContact);

#endif