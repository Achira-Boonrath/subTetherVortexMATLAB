// Created by Eleonora Botta, September 8, 2016
// Create a distance constraint, which has no effect until distance >= the specified distance
// Modified by LiamField 4/22/22 - modified to have the attachment point position. Input p1Pos and p2Pos as (0,0,0) for the default.

#include "MyLibraryNet/attachDistance.h"

#include <iostream>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

void attachDistance(VxSmartInterface<Assembly> assembly, VxSmartInterface<Part> p1, VxSmartInterface<Part> p2, VxVector3 p1Pos, VxVector3 p2Pos, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping)
{
    VxSmartInterface<DistanceJoint> d = VxExtensionFactory::create(DistanceJoint::kFactoryKey);
	d->inputAttachment1.part = p1;
	d->inputAttachment1.position = p1Pos;
	d->inputAttachment2.part = p2;
	d->inputAttachment2.position = p2Pos;
	d->inputDistance = MaxDistance; 
	d->inputDistanceEquation.relaxation.stiffness = threadStiffness;
	d->inputDistanceEquation.relaxation.damping = threadDamping;
	d->inputDistanceEquation.relaxation.enable = true;
    assembly->addConstraint(d);
    //return d;	  
}