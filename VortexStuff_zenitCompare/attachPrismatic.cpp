// Created by Eleonora Botta, September 12, 2016
// Create prismatic constraint between nodes to model bending stiffness 

#include "MyLibraryNet/attachPrismatic.h"

#include <iostream>
#include <VxDynamics/Prismatic.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

VxSmartInterface<Prismatic> attachPrismatic(VxSmartInterface<Assembly> assembly, VxSmartInterface<Part> p1, VxSmartInterface<Part> p2, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping, VxReal threadBendingStiffness, VxReal threadBendingDamping, VxReal threadTransverseStiffness, VxReal threadTransverseDamping)
{
    VxVector3 fromP2toP1 = p1->getVxPart()->getPosition() - p2->getVxPart()->getPosition();
    VxReal distance = fromP2toP1.normalize(); //make into unit vector and return original length
    std::cout << "From p2 to p1 = " << fromP2toP1.x() << "," << fromP2toP1.y() << " , " << fromP2toP1.z() << '\n'; 
            
	VxSmartInterface<Prismatic> d = VxExtensionFactory::create(Prismatic::kFactoryKey);
	d->inputAttachment1.part = p1;
    d->inputAttachment2.part = p2;
	d->getVxConstraint()->setPartAttachmentPosition(0,p1->getVxPart()->getPosition());
	d->getVxConstraint()->setPartAttachmentPosition(1,p2->getVxPart()->getPosition());
	//d->inputAttachment1.position = VxVector3(0.0, 0.0, 0.0);
   //d->inputAttachment1.primaryAxis = fromP2toP1;
    //d->inputAttachment1.secondaryAxis = p1->getVxPart()->getTransform().axis(2);
    //d->inputAttachment2.position = VxVector3(0.0, 0.0, 0.0);
    //d->inputAttachment2.primaryAxis = fromP2toP1;
    //d->inputAttachment2.secondaryAxis = p2->getVxPart()->getTransform().axis(2);

	//d->getVxConstraint()->setPartAttachmentPositionRel(0, Vx::VxVector3(0, 0, 0));
	//d->getVxConstraint()->setPartAttachmentPositionRel(1, Vx::VxVector3(0, 0, 0));
	d->getVxConstraint()->setPartAttachmentAxis(0,fromP2toP1);	
	d->getVxConstraint()->setPartAttachmentAxis(1,fromP2toP1);
   
    // allow angular motion with some stiffness and damping
	VxReal angularStiffness = threadBendingStiffness, angularDamping = threadBendingDamping; 
    d->getVxConstraint()->setRelaxationParameters(VxPrismatic::kConstraintA1,angularStiffness,angularDamping,0,true);  // rotation orthogonal to cable
    d->getVxConstraint()->setRelaxationParameters(VxPrismatic::kConstraintA2,angularStiffness,angularDamping,0,true);  // rotation orthogonal to cable
	// allow translational motion with some stiffness and damping
    VxReal translationalStiffness = threadTransverseStiffness, translationalDamping = threadTransverseDamping;
	d->getVxConstraint()->setRelaxationParameters(VxPrismatic::kConstraintP1,translationalStiffness,translationalDamping,0,true);  
    d->getVxConstraint()->setRelaxationParameters(VxPrismatic::kConstraintP2,translationalStiffness,translationalDamping,0,true);  

	assembly->addConstraint(d);
    return d;
}