// Created by Eleonora Botta, September 21, 2016
// Modified by Eleonora Botta, September 23, 2016: graphics for chaser
// Modified by Eleonora Botta, September 29, 2016: graphics for winch
// Modified by Eleonora Botta, March 3, 2017: if cable net model (netModel=2), don't show threads graphics
// Modified by Eleonora Botta, March 16, 2017: modified numbering of netModel: 0 = LP, 1 = cable-based. 
// Modified by Eleonora Botta, March 31, 2017: color the first corner mass in red, so that one is sure of direction of axes in OSG.
// Modified by Eleonora Botta, June 15, 2017: display the knots of the net if the cable-based model is used. 
// Modified by Eleonora Botta, May 14, 2020: added visualization of std closing mechanism
// Modified by Liam Field, March 15,2020: removed all net related content

#include "MyLibraryNet/displayScene.h"


#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxMath/Transformation.h>
#include <VxGraphics/ShapeGenerator.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/CollisionGeometry.h>

#include <iostream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// EB 14-05-2020 : add visualization of standard closing mechanisms
//void displayScene(VxGraphics::ShapeGenerator* mShapeGenerator, VxSmartInterface<Part>* partMatrix, VxSmartInterface<Part>* chaserPartPtr, float chaserSideLength, VxSmartInterface<Part>* tetheredTargetPartPtr)
void displayScene(VxGraphics::ShapeGenerator *mShapeGenerator, VxSmartInterface<Part>* partMatrix, VxSmartInterface<Part>* chaserPartPtr, float chaserSideLength, VxSmartInterface<Part>* tetheredTargetPartPtr, int numNode, int TetherType)
{
	//int numNode = 0;
	//int TetherType = 1;
	static Vx::VxColor threadColor = Vx::VxColor(0.0, 0.75, 0.0, 1.0);
	if (TetherType == 1) {
		if (numNode == 0)
		{
			//Getting Attachment Positions
			VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
			//VxSmartInterface<Assembly> Assem1 = Assem.getObject();
			VxSim::VxSmartInterface<DistanceJoint> v = Assem->getConstraints()[0];
			VxVector3 chaserAtt = v->inputAttachment1.position;
			VxVector3 targetAtt = v->inputAttachment2.position;



			Vx::VxQuaternion ChaserQ;
			(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
			VxVector3 chaserAttI = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();

			Vx::VxQuaternion TargetQ;
			(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
			VxVector3 targetAttI = TargetQ.rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();
			mShapeGenerator->drawLine(chaserAttI, targetAttI, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
		}
		else
		{
			for (int i = 0; i < numNode + 1; i++)
			{	//Getting Attachment Positions
				VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
				//VxSmartInterface<Assembly> Assem1 = Assem.getObject();
				VxSim::VxSmartInterface<DistanceJoint> v = Assem->getConstraints()[i];
				VxVector3 att1 = v->inputAttachment1.position;
				VxVector3 att2 = v->inputAttachment2.position;

				if (i == 0)
				{
					Vx::VxQuaternion ChaserQ;
					(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
					VxVector3 chaserAttI = ChaserQ.rotate(att1) + (*chaserPartPtr)->getVxPart()->getPosition();

					mShapeGenerator->drawLine(chaserAttI, partMatrix[i]->getVxPart()->getPosition(), VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);

				}
				else if (i == numNode)
				{
					Vx::VxQuaternion TargetQ;
					(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
					VxVector3 targetAttI = TargetQ.rotate(att2) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();
					mShapeGenerator->drawLine(partMatrix[i - 1]->getVxPart()->getPosition(), targetAttI, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);

				}
				else
				{
					mShapeGenerator->drawLine(partMatrix[i - 1]->getVxPart()->getPosition(), partMatrix[i]->getVxPart()->getPosition(), VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);

				}
			}

		}
	}
	else if (TetherType == 2) {

			//for (int i = 0; i < numNode + 1; i++)
			//{	//Getting Attachment Positions
			//	VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
			//	//VxSmartInterface<Assembly> Assem1 = Assem.getObject();
			//	VxSim::VxSmartInterface<DistanceJoint> v = Assem->getConstraints()[i];
			//	VxVector3 att1 = v->inputAttachment1.position;
			//	VxVector3 att2 = v->inputAttachment2.position;

			//	if (i == 0)
			//	{
			//		Vx::VxQuaternion ChaserQ;
			//		(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
			//		VxVector3 chaserAttI = ChaserQ.rotate(att1) + (*chaserPartPtr)->getVxPart()->getPosition();

			//		mShapeGenerator->drawLine(chaserAttI, partMatrix[i]->getVxPart()->getPosition(), VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);

			//	}
			//	else if (i == numNode)
			//	{
			//		Vx::VxQuaternion TargetQ;
			//		(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
			//		VxVector3 targetAttI = TargetQ.rotate(att2) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();
			//		mShapeGenerator->drawLine(partMatrix[i - 1]->getVxPart()->getPosition(), targetAttI, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);

			//	}
			//	else
			//	{
			//		mShapeGenerator->drawLine(partMatrix[i - 1]->getVxPart()->getPosition(), partMatrix[i]->getVxPart()->getPosition(), VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);

			//	}
			//}

			for (int i = 0; i < numNode + 4; i++)
			{	//Getting Attachment Positions
				VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
				//VxSmartInterface<Assembly> Assem1 = Assem.getObject();
				VxSim::VxSmartInterface<DistanceJoint> v = Assem->getConstraints()[i];
				VxVector3 att1 = v->inputAttachment1.position;
				VxVector3 att2 = v->inputAttachment2.position;

				if (i == 0)
				{
					Vx::VxQuaternion ChaserQ;
					(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
					VxVector3 chaserAttI = ChaserQ.rotate(att1) + (*chaserPartPtr)->getVxPart()->getPosition();

					mShapeGenerator->drawLine(chaserAttI, partMatrix[i]->getVxPart()->getPosition(), VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
				}
				else if (i == numNode)
				{
					Vx::VxQuaternion TargetQ;
					(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
					VxVector3 targetAttI = TargetQ.rotate(att2) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();
					mShapeGenerator->drawLine(partMatrix[numNode - 1]->getVxPart()->getPosition(), targetAttI, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
				}
				else if (i == numNode + 1)
				{
					Vx::VxQuaternion TargetQ;
					(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
					VxVector3 targetAttI = TargetQ.rotate(att2) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();
					mShapeGenerator->drawLine(partMatrix[numNode - 1]->getVxPart()->getPosition(), targetAttI, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
				}
				else if (i == numNode + 2)
				{
					Vx::VxQuaternion TargetQ;
					(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
					VxVector3 targetAttI = TargetQ.rotate(att2) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();
					mShapeGenerator->drawLine(partMatrix[numNode - 1]->getVxPart()->getPosition(), targetAttI, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
				}
				else if (i == numNode + 3)
				{
					Vx::VxQuaternion TargetQ;
					(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
					VxVector3 targetAttI = TargetQ.rotate(att2) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();
					mShapeGenerator->drawLine(partMatrix[numNode - 1]->getVxPart()->getPosition(), targetAttI, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
				}
				else
				{
					if (numNode > 1)
					{
					mShapeGenerator->drawLine(partMatrix[i - 1]->getVxPart()->getPosition(), partMatrix[i]->getVxPart()->getPosition(), VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
					}
				}
			}

		
	}
	

	//display chaser 
	
		static Vx::VxColor chaserColor = Vx::VxColor(0.0, 0.0, 0.6); // (red, green, blue, transparency)
		VxMath::Matrix44 pointTransform;
		VxSmartInterface<Part> chaserPart = *chaserPartPtr;
		chaserPart->getVxPart()->getPosition();
		pointTransform = VxMath::Transformation::createTranslation(chaserPart->getVxPart()->getPosition());
		VxReal3 chaserOrientation;
		chaserPart->getVxPart()->getOrientationEulerAngles(chaserOrientation);
		VxMath::Transformation::setRotation(pointTransform, chaserOrientation);
		VxMath::Transformation::setScale(pointTransform, Vx::VxVector3(chaserSideLength, chaserSideLength, chaserSideLength));
		mShapeGenerator->drawBox(pointTransform, VxGraphics::ShapeGenerator::DEFAULT_PASS, chaserColor);
	// display target
		static Vx::VxColor targetColor = Vx::VxColor(1.0, 0.0, 0.0,1.0); // (red, green, blue, transparency)
		VxMath::Matrix44 pointTransform2;
		VxSmartInterface<Part> tetheredTargetPart = *tetheredTargetPartPtr;
		tetheredTargetPart->getVxPart()->getPosition();
		pointTransform2 = VxMath::Transformation::createTranslation(tetheredTargetPart->getVxPart()->getPosition());
		VxReal3 tetheredTargetOrientation;
		tetheredTargetPart->getVxPart()->getOrientationEulerAngles(tetheredTargetOrientation);
		VxMath::Transformation::setRotation(pointTransform2, tetheredTargetOrientation);
		VxMath::Transformation::setScale(pointTransform2, Vx::VxVector3(1.25, 1.75, 1.25));
		mShapeGenerator->drawBox(pointTransform2, VxGraphics::ShapeGenerator::DEFAULT_PASS, targetColor);
	

}