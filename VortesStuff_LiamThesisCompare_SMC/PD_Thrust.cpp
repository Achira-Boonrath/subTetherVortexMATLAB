// Created by Liam Field 2/9/2023
//  PD control implementation: distance between tether and center node

#include "MyLibraryNet/PD_Thrust.h"
#include "MyLibraryNet/QuatMat.h"


#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>
#include <Vx/VxConstraint.h>
#include <Vx/VxCylindrical.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxMath/Transformation.h>
#include <VxMath/VxMath.h>
#include <Vx/VxQuaternion.h>

#include <VxSim/IMobile.h>	// this is needed by VxSmartInterface<IExtension> cableExtension


#include <VxData/Vector.h>
#include <Vx/VxSmartPtr.h>


#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;


std::vector<double> PD_Thrust(VxSmartInterface<Part>* p1, VxSmartInterface<Part>* chaserPartPtr, std::array<double,4> PD_States, VxSmartInterface<DistanceJoint> tetherJoints[], int Direction, const int numnodes)
{
	// Pulling things out
	VxQuaternion ChaserQ;
	(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
	VxVector3 Att_Chaser = tetherJoints[0]->inputAttachment1.position;
	const double tetherLength = tetherJoints[0]->inputDistance.getValue();
	VxVector3 chaserAngVel = ChaserQ.getInverse().rotate((*chaserPartPtr)->getVxPart()->getAngularVelocity());
	
	VxQuaternion TargetQ;
	(*p1)->getVxPart()->getOrientationQuaternion(TargetQ);
	VxVector3 Att_Target = tetherJoints[0]->inputAttachment2.position;
	VxVector3 targetAngVel = TargetQ.getInverse().rotate((*p1)->getVxPart()->getAngularVelocity());

	// Correct, ChaserQ rotates from e
	//VxVector3 bodyAngVel = ChaserQ.getInverse().rotate(angVel);
	// cross product i.e. G.cross(v) is this order: G x v

	// Tether Length: Chaser->Target direction
	VxVector3 L = -((*chaserPartPtr)->getVxPart()->getPosition() + ChaserQ.rotate(Att_Chaser) - (*p1)->getVxPart()->getPosition()- TargetQ.rotate(Att_Target));
	// Process variable
	VxReal errorVar = PD_States[0] + (numnodes+1) * tetherLength - L.norm();
	// Process variable derivative
	VxReal errorVarDeriv = -(-(*chaserPartPtr)->getVxPart()->getLinearVelocity() - ChaserQ.rotate(chaserAngVel.cross(Att_Chaser)) + (*p1)->getVxPart()->getLinearVelocity()-TargetQ.rotate(targetAngVel.cross(Att_Target))).dot(L/L.norm());

	VxVector3 InputThrust;
	//TestThrustDirection
	 	   /*auto TestThrustDirection = */
	// Calculated magnitude and application direction of thrust
	if (Direction == 0)
	{
		InputThrust = -(PD_States[1] * errorVar + PD_States[2] * errorVarDeriv) * ((*chaserPartPtr)->getVxPart()->getLinearVelocity() / (*chaserPartPtr)->getVxPart()->getLinearVelocity().norm());
	}
	else
	{
		InputThrust = -(PD_States[1] * errorVar + PD_States[2] * errorVarDeriv) * (L / L.norm());
	}
	std::vector<double> OutputPD;
	OutputPD.push_back(errorVar);
	OutputPD.push_back(errorVarDeriv);
	OutputPD.push_back(InputThrust.x());
	OutputPD.push_back(InputThrust.y());
	OutputPD.push_back(InputThrust.z());
	/////// Testing Region
	/*auto LvecN = L.norm()-tetherLength;
	auto Chs2 = (*chaserPartPtr)->getVxPart()->getPosition() + LvecN*L/L.norm();
	VxVector3 L2 = -(Chs2 + ChaserQ.rotate(Att_Chaser) - (*p1)->getVxPart()->getPosition() - TargetQ.rotate(Att_Target));
	auto LvecN2 = L2.norm() - tetherLength;
	auto chsVel = (*chaserPartPtr)->getVxPart()->getLinearVelocity();
	auto chsAngVV = ChaserQ.rotate(chaserAngVel.cross(Att_Chaser));
	auto p1Pos = (*p1)->getVxPart()->getPosition();*/
	
	return OutputPD;
}