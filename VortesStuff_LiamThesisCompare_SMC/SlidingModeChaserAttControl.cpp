// Created by Liam Field 3/21/22
// Sliding mode controller for chaser attitude

#include "MyLibraryNet/SlidingModeChaserAttControl.h"
#include "MyLibraryNet/QuatMat.h"
#include "MyLibraryNet/ErrorQuatCalc.h"


#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>
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

VxVector3 SlidingModeChaserAttControl(VxSmartInterface<Part>* chaserPartPtr, VxSmartInterface<Part>* tetheredTargetPartPtr,VxVector3 C, VxVector3 K,VxVector3 desiredAngVel,double delta,FILE* outSMC)
{
	VxVector3 angVel = (*chaserPartPtr)->getVxPart()->getAngularVelocity();
	VxQuaternion ChaserQ;
		(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);

		// VxVector3 bodyAngVel2 = ChaserQ.rotate(angVel);

		// Correct, ChaserQ rotates from inertial
		VxVector3 bodyAngVel = ChaserQ.getInverse().rotate(angVel);

		//Getting Attachment Positions - Non-hardcoded version for distancejointTether
		//VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
		//VxSim::VxSmartInterface<DistanceJoint> v = Assem->getConstraints()[0];
		//VxVector3 chaserAtt = v->inputAttachment1.position;
		//VxVector3 targetAtt = v->inputAttachment2.position;

		// Works for tether as a cable and distance joint, should be changed to load this stuff in here
		VxVector3 chaserAtt = VxVector3(.5,0,0);
		VxVector3 targetAtt = VxVector3(0, -1.75/2, 0);
		//Vx::VxQuaternion ChaserQ;
		VxVector3 HHH = ChaserQ.rotate(chaserAtt);
		//(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
		VxVector3 chaserAttPos = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();

		Vx::VxQuaternion TargetQ;
		(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
		VxVector3 targetAttPos = TargetQ.rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();


		VxVector3 xc = -(chaserAttPos - targetAttPos) / (chaserAttPos - targetAttPos).norm();
		VxVector3 zc = xc.cross((*chaserPartPtr)->getVxPart()->getPosition()) / (xc.cross((*chaserPartPtr)->getVxPart()->getPosition())).norm();
		VxVector3 yc = zc.cross(xc) / (zc.cross(xc)).norm();

		VxMath::Matrix44 RotMat = VxMath::Transformation::createRotationFromAxes(xc, yc, zc);
		VxQuaternion DesQ = VxMath::Transformation::getRotationAsQuaternion(RotMat);
		DesQ.normalize();



		// Desired Quaternion Sigma matrix
		VxMath::Matrix44 QMatD = QuatMat(DesQ, 1);
		// Chaser Quaternion Sigma Matrix
		VxMath::Matrix44 QMatC = QuatMat(ChaserQ, 1);
		

		// NEW METHOD:
		// Error Quaternion
		VxVector3 qe_vecMID = ErrorCalcQuat(DesQ, ChaserQ);
		double qe_scalar = DesQ.dot(ChaserQ);
		VxQuaternion qe = VxQuaternion(qe_scalar, qe_vecMID.x(), qe_vecMID.y(), qe_vecMID.z());
		VxMath::Matrix44 QMatE = QuatMat(qe, 1);
		VxQuaternion qdotE = 0.5 * operator*(QMatE, VxMath::VxVector4(bodyAngVel, 0));




		//// OLD METHOD:
		//auto testMAT = QMatD.transpose();
		//// Error Quaternion
		//VxQuaternion qe_vecMID =  operator*(QMatD.transpose(), ChaserQ.operator const double *());
		//double qe_scalar = DesQ.dot(ChaserQ);
		//VxQuaternion qe = VxQuaternion(qe_scalar, qe_vecMID.w(), qe_vecMID.x(), qe_vecMID.y());
		//VxMath::Matrix44 QMatE = QuatMat(qe, 1);
		//VxQuaternion qdotE = 0.5 * operator*(QMatE, VxMath::VxVector4(bodyAngVel, 0));
		

		// Sliding vector
		VxVector3 s = (bodyAngVel-desiredAngVel)+VxVector3(C[0]*qe.x(),C[1]*qe.y(),C[2]*qe.z()).operator*=(VxSign(qe_scalar));

		// Saturation Function
		VxVector3 sat;
		for (int i = 0; i < 3; i++)
		{
			sat[i] = std::min(std::max(double(s[i]/delta), double(-1)), double(1));
		}

		// Getting the Inertia Tensor
		Vx::VxMassProperties MC;
		MC = (*chaserPartPtr)->getVxPart()->getMassProperties();
		Vx::VxInertiaTensor H;
		MC.getInertiaTensorLocal(H);

		// Creating Inertia Matrix
		VxMath::Matrix44 InertiaMatrix;
		InertiaMatrix.operator()(1, 1) = H(0, 0);
		InertiaMatrix.operator()(2, 2) = H(1, 1);
		InertiaMatrix.operator()(3, 3) = H(2, 2);

		// Calculating the angular velocity cross product		
		VxVector3 AngVelInertia;
		for (int i = 0; i < 3; i++)
		{
			AngVelInertia[i] = bodyAngVel[i]*H(i, i);
		}

		VxVector3 AngCrossInertiaAngVel = bodyAngVel.cross(AngVelInertia);


		// Calculating variable f and resizing
		VxMath::VxVector4 f = -operator*(InertiaMatrix, VxMath::VxVector4(AngCrossInertiaAngVel, 0));
		VxVector3 functionAngVel = f.xyz();

		// Input before inertia multiplication
		VxVector3 Umid = -functionAngVel - VxVector3(C[0] * qdotE.x(), C[1] * qdotE.y(), C[2] * qdotE.z()).operator*=(VxSign(qe_scalar))-VxVector3(K[0] * sat.x(), K[1] * sat.y(), K[2] * sat.z());
		
		// final control input calculation
		VxVector3 U;
		for (int i = 0; i < 3; i++)
		{
			U[i] = H(i, i)*Umid[i];
		}

		VxVector3 Uout = ChaserQ.rotate(U);

		
		//fprintf(outSMC, "%f \t %f \t %f \t %f \t %f \t %f \t  %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f", bodyAngVel.x(), bodyAngVel.y(), bodyAngVel.z(), chaserAttPos.x(), chaserAttPos.y(), chaserAttPos.z(), qe.w(), qe.x(), qe.y(), qe.z()
		//	, U.x(), U.y(), U.z(), s.x(), s.y(), s.z(), sat.x(), sat.y(), sat.z());
	
		//fprintf(outSMC, "\n");
		
		return Uout;



//		qe_vec[0] = qe_s;
//		VxMath::VxVector4 J = DesQ.operator double *();
//		VxMath::VxVector4 B = operator*(RotMat, VxMath::VxVector4(xc, 0));
//		VxQuaternion qe_vec = operator*(QMatD.transpose(), DesQ.operator double* ());

//		VxVector3 N = B.xyz();
		
		


		// Scalar Last
		//QMatD.operator()(0, 0) = DesQ.w();
		//QMatD.operator()(1, 1) = DesQ.w();
		//QMatD.operator()(2, 2) = DesQ.w();
		//QMatD.operator()(0, 1) = -DesQ.z();
		//QMatD.operator()(1, 0) = DesQ.z();
		//QMatD.operator()(0, 2) = DesQ.y();
		//QMatD.operator()(2, 0) = -DesQ.y();
		//QMatD.operator()(1, 2) = -DesQ.x();
		//QMatD.operator()(2, 1) = DesQ.x();

		//QMatD.operator()(3, 0) = -DesQ.x();
		//QMatD.operator()(3, 1) = -DesQ.y();
		//QMatD.operator()(3, 2) = -DesQ.z();
		//QMatD.operator()(3, 3) = 0;
		//////////////









}


VxVector3 SlidingModeChaserAttControl_ST(VxSmartInterface<Part>* chaserPartPtr, VxSmartInterface<Part> tetheredTargetPartPtr, VxVector3 C, VxVector3 K, VxVector3 desiredAngVel, double delta, FILE* outSMC)
{
	VxVector3 angVel = (*chaserPartPtr)->getVxPart()->getAngularVelocity();
	VxQuaternion ChaserQ;
	(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);

	// VxVector3 bodyAngVel2 = ChaserQ.rotate(angVel);

	// Correct, ChaserQ rotates from inertial
	VxVector3 bodyAngVel = ChaserQ.getInverse().rotate(angVel);

	//Getting Attachment Positions - Non-hardcoded version for distancejointTether
	//VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
	//VxSim::VxSmartInterface<DistanceJoint> v = Assem->getConstraints()[0];
	//VxVector3 chaserAtt = v->inputAttachment1.position;
	//VxVector3 targetAtt = v->inputAttachment2.position;

	// Works for tether as a cable and distance joint, should be changed to load this stuff in here
	VxVector3 chaserAtt = VxVector3(.5, 0, 0);
	//VxVector3 targetAtt = VxVector3(0, -1.75 / 2, 0);
	VxVector3 targetAtt = VxVector3(0, 0, 0);

	// Liam Original
	////Vx::VxQuaternion ChaserQ;
	//VxVector3 HHH = ChaserQ.rotate(chaserAtt);
	////(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
	//VxVector3 chaserAttPos = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();
	//Vx::VxQuaternion TargetQ;
	//(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
	//VxVector3 targetAttPos = TargetQ.rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();
	//VxVector3 xc = -(chaserAttPos - targetAttPos) / (chaserAttPos - targetAttPos).norm();
	//VxVector3 zc = xc.cross((*chaserPartPtr)->getVxPart()->getPosition()) / (xc.cross((*chaserPartPtr)->getVxPart()->getPosition())).norm();
	//VxVector3 yc = zc.cross(xc) / (zc.cross(xc)).norm();
	//VxMath::Matrix44 RotMat = VxMath::Transformation::createRotationFromAxes(xc, yc, zc);
	//VxQuaternion DesQ = VxMath::Transformation::getRotationAsQuaternion(RotMat);
	//DesQ.normalize();

	// AB New des Qar
	//Vx::VxQuaternion ChaserQ;
	VxVector3 HHH = ChaserQ.rotate(chaserAtt);
	//(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
	VxVector3 chaserAttPos = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();
	Vx::VxQuaternion TargetQ;
	tetheredTargetPartPtr->getVxPart()->getOrientationQuaternion(TargetQ);
	VxVector3 targetAttPos = TargetQ.rotate(targetAtt) + tetheredTargetPartPtr->getVxPart()->getPosition();
	//VxVector3 zc = (chaserAttPos - targetAttPos) / (chaserAttPos - targetAttPos).norm();
	//VxVector3 xc = zc.cross((*chaserPartPtr)->getVxPart()->getPosition()) / (zc.cross((*chaserPartPtr)->getVxPart()->getPosition())).norm();
	//VxVector3 yc = zc.cross(xc) / (zc.cross(xc)).norm();
	VxVector3 xc = -(chaserAttPos - targetAttPos) / (chaserAttPos - targetAttPos).norm();
	VxVector3 zc = xc.cross((*chaserPartPtr)->getVxPart()->getPosition()) / (xc.cross((*chaserPartPtr)->getVxPart()->getPosition())).norm();
	VxVector3 yc = zc.cross(xc) / (zc.cross(xc)).norm();
	VxMath::Matrix44 RotMat = VxMath::Transformation::createRotationFromAxes(xc, yc, zc);
	VxQuaternion DesQ = VxMath::Transformation::getRotationAsQuaternion(RotMat);
	DesQ.normalize();

	// Liam Original
	//// Desired Quaternion Sigma matrix
	//VxMath::Matrix44 QMatD = QuatMat(DesQ, 1);
	//// Chaser Quaternion Sigma Matrix
	//VxMath::Matrix44 QMatC = QuatMat(ChaserQ, 1);
	//// NEW METHOD:
	//// Error Quaternion
	//VxVector3 qe_vecMID = ErrorCalcQuat(DesQ, ChaserQ);
	//double qe_scalar = DesQ.dot(ChaserQ);
	//VxQuaternion qe = VxQuaternion(qe_scalar, qe_vecMID.x(), qe_vecMID.y(), qe_vecMID.z());
	//VxMath::Matrix44 QMatE = QuatMat(qe, 1);
	//VxQuaternion qdotE = 0.5 * operator*(QMatE, VxMath::VxVector4(bodyAngVel, 0));

	// AB New des Qar
	// Desired Quaternion Sigma matrix
	VxMath::Matrix44 QMatD = QuatMat(DesQ, 1);
	// Chaser Quaternion Sigma Matrix
	VxMath::Matrix44 QMatC = QuatMat(ChaserQ, 1);
	// Error Quaternion
	VxQuaternion qe_vecMID = operator*(QMatD.transpose(), ChaserQ.operator const double* ());
	double qe_scalar = DesQ.dot(ChaserQ);
	VxQuaternion qe = VxQuaternion(qe_scalar, qe_vecMID.w(), qe_vecMID.x(), qe_vecMID.y());
	VxMath::Matrix44 QMatE = QuatMat(qe, 1);
	VxQuaternion qdotE = 0.5 * operator*(QMatE, VxMath::VxVector4(bodyAngVel, 0));


	// Sliding vector
	VxVector3 s = (bodyAngVel - desiredAngVel) + VxVector3(C[0] * qe.x(), C[1] * qe.y(), C[2] * qe.z()).operator*=(VxSign(qe_scalar));

	// Saturation Function
	VxVector3 sat;
	for (int i = 0; i < 3; i++)
	{
		sat[i] = std::min(std::max(double(s[i] / delta), double(-1)), double(1));
	}

	// Getting the Inertia Tensor
	Vx::VxMassProperties MC;
	MC = (*chaserPartPtr)->getVxPart()->getMassProperties();
	Vx::VxInertiaTensor H;
	MC.getInertiaTensorLocal(H);

	// Creating Inertia Matrix
	VxMath::Matrix44 InertiaMatrix;
	InertiaMatrix.operator()(1, 1) = H(0, 0);
	InertiaMatrix.operator()(2, 2) = H(1, 1);
	InertiaMatrix.operator()(3, 3) = H(2, 2);

	// Calculating the angular velocity cross product		
	VxVector3 AngVelInertia;
	for (int i = 0; i < 3; i++)
	{
		AngVelInertia[i] = bodyAngVel[i] * H(i, i);
	}

	VxVector3 AngCrossInertiaAngVel = bodyAngVel.cross(AngVelInertia);


	// Calculating variable f and resizing
	VxMath::VxVector4 f = -operator*(InertiaMatrix, VxMath::VxVector4(AngCrossInertiaAngVel, 0));
	VxVector3 functionAngVel = f.xyz();

	// Input before inertia multiplication
	VxVector3 Umid = -functionAngVel - VxVector3(C[0] * qdotE.x(), C[1] * qdotE.y(), C[2] * qdotE.z()).operator*=(VxSign(qe_scalar)) - VxVector3(K[0] * sat.x(), K[1] * sat.y(), K[2] * sat.z());

	// final control input calculation
	VxVector3 U;
	for (int i = 0; i < 3; i++)
	{
		U[i] = H(i, i) * Umid[i];
	}

	VxVector3 Uout = ChaserQ.rotate(U);


	//fprintf(outSMC, "%f \t %f \t %f \t %f \t %f \t %f \t  %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f", bodyAngVel.x(), bodyAngVel.y(), bodyAngVel.z(), chaserAttPos.x(), chaserAttPos.y(), chaserAttPos.z(), qe.w(), qe.x(), qe.y(), qe.z()
	//	, U.x(), U.y(), U.z(), s.x(), s.y(), s.z(), sat.x(), sat.y(), sat.z());

	//fprintf(outSMC, "\n");

	return Uout;

}