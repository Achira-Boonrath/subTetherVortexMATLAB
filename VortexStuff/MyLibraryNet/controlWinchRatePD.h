// Created by Eleonora Botta, February 13, 2017
// Modified by Eleonora Botta, February 17, 2017: possibility to have 2 tethers configuration. Needs a different allowable length

#ifndef CONTROLWINCHRATEPD_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define CONTROLWINCHRATEPD_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Hinge.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxHinge.h>
#include <CableSystems/DynamicsICD.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
//void controlWinchRatePD(VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr);

//float controlWinchRatePD(VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxSmartInterface<IExtension> cableExtension);
float controlWinchRatePD(VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension );

float controlWinchRatePDon4Tethers(VxReal minAllowableLength, VxReal maxAllowableTension, VxReal tethersNb, int indexCenterPart, int indexThirdPart, int indexTwoThirdsPart, VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension, VxReal cableMinLength );

//float controlWinchRatePDon4TethersWith3Rings(VxReal maxAllowableTension, VxReal tethersNb, int indexCenterPart, VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension );
//float controlWinchRatePDon4TethersWith3Rings(VxReal maxAllowableTension, VxReal tethersNb, int indexCenterPart, VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension, VxReal cableMinLength );
float controlWinchRatePDon4TethersWith3Rings(VxReal minAllowableLength, VxReal maxAllowableTension, VxReal tethersNb, int indexCenterPart, VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension, VxReal cableMinLength );

float controlWinchRatePDon2Tethers(VxReal minAllowableLength, VxReal maxAllowableTension, VxReal tethersNb, int indexCenterPart, VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension, VxReal cableMinLength );

#endif