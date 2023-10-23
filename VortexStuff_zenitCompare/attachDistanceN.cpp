// Created by Eleonora Botta, September 8, 2016
// Create a distance constraint, which has no effect until distance >= the specified distance

#include "MyLibraryNet/attachDistanceN.h"

#include "MyLibraryNet/attachDistance.h"
#include <iostream>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Sphere.h>
#include <Vx/VxPart.h>
#include <VxMath/VxMath.h>
#include <VxMath/Transformation.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxSim/VxSmartInterface.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

void attachDistanceN(VxSmartInterface<Assembly> assembly, VxSmartInterface<Part> chaser, VxSmartInterface<Part> target, VxSmartInterface<Part> *partMatrix, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping,int numNode, float partMass)
{
	//VxSmartInterface<Part> partMatrix;

	//Liam Original
	//for (int i = 0; i < numNode; i++)
	//{
	//	// part collision geometry
	//	VxSmartInterface<Sphere> sphere = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey);
	//	VxReal sphereRadius = .1; // x 10 for visualization only;
	//	sphere->parameterRadius = sphereRadius;
	//	//fprintf(FileLog, "\nThe radius of the sphere of index  %i,%i is %g m \n", i,j,sphereRadius );

	//	// create part
	//	VxSmartInterface<Part> p = VxExtensionFactory::create(PartICD::kFactoryKey);
	//	p->parameterMassPropertiesContainer.mass = partMass;
	//	VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey);
	//	cg = sphere;
	//	p->addCollisionGeometry(cg);
	//	p->autoComputeInertiaAndCOM();
	//	VxVector3 pos = VxVector3(8,6,7);
	//	p->setLocalTransform(VxMath::Transformation::createTranslation(pos));
	//	p->setLinearVelocity(Vx::VxVector3());
	//	partMatrix[i] = p;
	//	//std::cout << "The position of the sphere of index " << i << "," << j << " is " << pos.x() << ", " << pos.y() << ", " << pos.z() << " m" << '\n'; 
	//	assembly->addPart(p);
	//}

	//AB test
	for (int i = 0; i < numNode; i++)
	{
		// part collision geometry
		VxSmartInterface<Sphere> sphere = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey);
		VxReal sphereRadius = .1; // x 10 for visualization only;
		sphere->parameterRadius = sphereRadius;
		//fprintf(FileLog, "\nThe radius of the sphere of index  %i,%i is %g m \n", i,j,sphereRadius );

		// create part
		VxSmartInterface<Part> p = VxExtensionFactory::create(PartICD::kFactoryKey);
		//p->parameterMassPropertiesContainer.mass = partMass;
		p->parameterMassPropertiesContainer.mass = 1.0;
		VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey);
		cg = sphere;
		p->addCollisionGeometry(cg);
		p->autoComputeInertiaAndCOM();
		//VxVector3 pos = VxVector3(8, 6, 7);

		//AB, pos interpolation
		VxVector3 pos_adj_1 = chaser->getVxPart()->getPosition(); //
		VxVector3 pos_adj_2 = target->getVxPart()->getPosition(); //
		//VxVector3 pos = pos_adj_1 + ((i + 1) * (pos_adj_2 - pos_adj_1) / (numNode + 1 + 1)); //linear interpolation of position based on knots
		VxVector3 pos = pos_adj_1 + ((i + 1) * (pos_adj_2 - pos_adj_1) / (numNode + 1)); //linear interpolation of position based on knots


		p->setLocalTransform(VxMath::Transformation::createTranslation(pos));
		p->setLinearVelocity(Vx::VxVector3(0.0,0.0,0.0));
		partMatrix[i] = p;
		//std::cout << "The position of the sphere of index " << i << "," << j << " is " << pos.x() << ", " << pos.y() << ", " << pos.z() << " m" << '\n'; 
		assembly->addPart(p);

	}

	//Liam Original
	//for (int i = 0; i < numNode+1; i++)
	//{	
	//	if (i == 0)
	//	{
	//		 attachDistance(assembly,chaser,partMatrix[i],VxVector3(.5,0,0),VxVector3(0,0,0),MaxDistance,threadStiffness,threadDamping);
	//		// VxSmartInterface<DistanceJoint> d =


	//	}
	//	else if (i == numNode)
	//	{
	//		 attachDistance(assembly, partMatrix[i-1], target, VxVector3(0, 0, 0), VxVector3(0, -1.75/2, 0), MaxDistance, threadStiffness, threadDamping);
	//	}
	//	else
	//	{
	//		 attachDistance(assembly, partMatrix[i - 1], partMatrix[i], VxVector3(0, 0, 0), VxVector3(0, 0, 0), MaxDistance, threadStiffness, threadDamping);

	//	}
	//}

	//AB test
	for (int i = 0; i < numNode + 1; i++)
	{
		if (i == 0)
		{
			VxVector3 pos1 = chaser->getVxPart()->getPosition(); //
			VxVector3 pos2 = partMatrix[i]->getVxPart()->getPosition(); //
			MaxDistance = (pos1 - pos2).norm();

			//original
			attachDistance(assembly, chaser, partMatrix[i], VxVector3(.5, 0, 0), VxVector3(0, 0, 0), MaxDistance, threadStiffness, threadDamping);

			// VxSmartInterface<DistanceJoint> d =


		}
		else if (i == numNode)
		{
			VxVector3 pos1 = partMatrix[numNode - 1]->getVxPart()->getPosition(); //
			VxVector3 pos2 = target->getVxPart()->getPosition(); //
			MaxDistance = 1.0 * (pos1 - pos2).norm();

			VxReal targetMass = target->getVxPart()->getMass(); //

			attachDistance(assembly, partMatrix[i - 1], target, VxVector3(0, 0, 0), VxVector3(0, -1.75 / 2, 0), MaxDistance, threadStiffness, threadDamping);
		}
		else
		{

			attachDistance(assembly, partMatrix[i - 1], partMatrix[i], VxVector3(0, 0, 0), VxVector3(0, 0, 0), MaxDistance, threadStiffness, threadDamping);

		}
	}
	
}

void attachDistanceN_ST(VxSmartInterface<Assembly> assembly, VxSmartInterface<Part> chaser, VxSmartInterface<Part> target, VxSmartInterface<Part>* partMatrix, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping, int numNode, float partMass)
{
	//VxSmartInterface<Part> partMatrix;
	//numNode = 1;


	for (int i = 0; i < numNode; i++)
	{
		// part collision geometry
		VxSmartInterface<Sphere> sphere = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey);
		VxReal sphereRadius = .1; // x 10 for visualization only;
		sphere->parameterRadius = sphereRadius;
		//fprintf(FileLog, "\nThe radius of the sphere of index  %i,%i is %g m \n", i,j,sphereRadius );

		// create part
		VxSmartInterface<Part> p = VxExtensionFactory::create(PartICD::kFactoryKey);
		//p->parameterMassPropertiesContainer.mass = partMass;
		//p->parameterMassPropertiesContainer.mass = 1.0;
		p->parameterMassPropertiesContainer.mass = 2.732628345489502;
		VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey);
		cg = sphere;
		p->addCollisionGeometry(cg);
		p->autoComputeInertiaAndCOM();
		//VxVector3 pos = VxVector3(8, 6, 7);

		//AB, pos interpolation
		VxVector3 pos_adj_1 = chaser->getVxPart()->getPosition(); //
		VxVector3 pos_adj_2 = target->getVxPart()->getPosition(); //
		//VxVector3 pos = pos_adj_1 + ((i + 1) * (pos_adj_2 - pos_adj_1) / (numNode + 1 + 1)); //linear interpolation of position based on knots
		VxVector3 pos = pos_adj_1 + ((i + 1) * (pos_adj_2 - pos_adj_1) / (numNode + 1)); //linear interpolation of position based on knots


		p->setLocalTransform(VxMath::Transformation::createTranslation(pos));

		//p->setLocalTransform(VxMath::Transformation::createTranslation(Vx::VxVector3(5069371.4573326809, -947653.47841382329, -4545448.5652576294)));
		//p->setLocalTransform(VxMath::Transformation::createTranslation(Vx::VxVector3(5.069348697744000e+06, -9.476660265470000e+05, -4.545464763125000e+06)));
		//p->setLinearVelocity(Vx::VxVector3(0.0, 0.0, 0.0));//AB, no grav

		VxVector3 vel_adj_1 = chaser->getVxPart()->getLinearVelocity(); //
		VxVector3 vel_adj_2 = target->getVxPart()->getLinearVelocity(); //
		p->setLinearVelocity(vel_adj_2);//AB, grav

		partMatrix[i] = p;
		//std::cout << "The position of the sphere of index " << i << "," << j << " is " << pos.x() << ", " << pos.y() << ", " << pos.z() << " m" << '\n'; 
		assembly->addPart(p);

	}

	//AB, compare with MATLAB
	//threadStiffness = 1000.0;
	//threadDamping = threadStiffness/10.0;
	//MaxDistance = 15.6875;

	//AB, Replicate Zenit Capture
	threadStiffness = 10000.0;
	threadDamping = threadStiffness / 2.0;
	MaxDistance = 7.51;
	VxReal MaxDistanceST = 9.0;

	int fixedMaxDist = 1;
	//threadDamping = 1000;
	for (int i = 0; i < numNode + 4; i++)
	{
		if (i == 0)
		{

			VxVector3 pos1 = chaser->getVxPart()->getPosition(); //
			VxVector3 pos2 = partMatrix[i]->getVxPart()->getPosition(); //

			if (fixedMaxDist == 0)
			{
				MaxDistance = (pos1 - pos2).norm();
			}
			//original
			//attachDistance(assembly, chaser, partMatrix[i], VxVector3(.5, 0, 0), VxVector3(0, 0, 0), MaxDistance, threadStiffness, threadDamping);

			//AB, Replicate Zenit Capture
			attachDistance(assembly, chaser, partMatrix[i], VxVector3(0, 0, 0.0), VxVector3(0, 0, 0), MaxDistance, threadStiffness, threadDamping);
			//attachDistance(assembly, chaser, partMatrix[i], VxVector3(0, 0, -0.5), VxVector3(0, 0, 0), MaxDistance, threadStiffness, threadDamping);

		}
		else if (i == numNode)
		{
			VxVector3 pos1 = partMatrix[numNode - 1]->getVxPart()->getPosition(); //
			VxVector3 pos2 = target->getVxPart()->getPosition(); //

			if (fixedMaxDist == 0)
			{
				MaxDistance = 1.0 * (pos1 - pos2).norm();
			}
			VxReal threadStiffnessST = threadStiffness / 1.0;

			//Liam Orientation
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-1.25 / 2.0, -1.75 / 2, -1.25 / 2.0), MaxDistance, threadStiffnessST, threadDamping);

			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-1.75 / 2, -1.25 / 2.0, -1.25 / 2.0), MaxDistance, threadStiffnessST, threadDamping);
			
			attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-3.25 / 2.0, -3.25 / 2.0, -11.0 / 2.0), MaxDistanceST, threadStiffnessST, threadDamping);
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-3.25 / 2.0, 0, 0), MaxDistanceST, threadStiffnessST, threadDamping);
		}
		else if (i == numNode + 1 )
		{
			VxVector3 pos1 = partMatrix[numNode - 1]->getVxPart()->getPosition(); //
			VxVector3 pos2 = target->getVxPart()->getPosition(); //

			if (fixedMaxDist == 0)
			{
				MaxDistance = 1.0 * (pos1 - pos2).norm();
			}
			VxReal threadStiffnessST = threadStiffness / 1.0;
			//Liam
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-1.25 / 2.0, -1.75 / 2, 1.25 / 2.0), MaxDistance, threadStiffnessST, threadDamping);
			
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-1.75 / 2, -1.25 / 2.0, 1.25 / 2.0), MaxDistance, threadStiffnessST, threadDamping);

			//AB, Replicate Zenit Capture
			attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-3.25 / 2.0, -3.25 / 2.0, 11.0 / 2.0), MaxDistanceST, threadStiffnessST, threadDamping);
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-3.25 / 2.0, 0, 0), MaxDistanceST, threadStiffnessST, threadDamping);
		}
		else if (i == numNode + 2)
		{
			VxVector3 pos1 = partMatrix[numNode - 1]->getVxPart()->getPosition(); //
			VxVector3 pos2 = target->getVxPart()->getPosition(); //

			if (fixedMaxDist == 0)
			{
				MaxDistance = 1.0 * (pos1 - pos2).norm();
			}
			VxReal threadStiffnessST = threadStiffness / 1.0;

			//Liam
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(1.25 / 2.0, -1.75 / 2, -1.25 / 2.0), MaxDistance, threadStiffnessST, threadDamping);
		
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-1.75 / 2, 1.25 / 2.0, -1.25 / 2.0), MaxDistance, threadStiffnessST, threadDamping);

			//AB, Replicate Zenit Capture
			attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-3.25 / 2.0, 3.25 / 2.0, -11.0 / 2.0), MaxDistanceST, threadStiffnessST, threadDamping);
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-3.25 / 2.0, 0, 0), MaxDistanceST, threadStiffnessST, threadDamping);
		}
		else if (i == numNode + 3)
		{
			VxVector3 pos1 = partMatrix[numNode - 1]->getVxPart()->getPosition(); //
			VxVector3 pos2 = target->getVxPart()->getPosition(); //

			if (fixedMaxDist == 0)
			{
				MaxDistance = 1.0 * (pos1 - pos2).norm();
			}
			VxReal threadStiffnessST = threadStiffness / 1.0;

			//Liam
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(1.25 / 2.0, -1.75 / 2, 1.25 / 2.0), MaxDistance, threadStiffnessST, threadDamping);
		
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-1.75 / 2, 1.25 / 2.0, 1.25 / 2.0), MaxDistance, threadStiffnessST, threadDamping);

			//AB, Replicate Zenit Capture
			attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-3.25 / 2.0, 3.25 / 2.0, 11.0 / 2.0), MaxDistanceST, threadStiffnessST, threadDamping);
			//attachDistance(assembly, partMatrix[numNode - 1], target, VxVector3(0, 0, 0), VxVector3(-3.25 / 2.0, 0, 0), MaxDistanceST, threadStiffnessST, threadDamping);
		}
		else
		{
			if (numNode > 1)
			{
				VxVector3 pos1 = partMatrix[numNode - 1]->getVxPart()->getPosition(); //
				VxVector3 pos2 = partMatrix[i]->getVxPart()->getPosition(); //

				if (fixedMaxDist == 0)
				{
					MaxDistance = (pos1 - pos2).norm();
				}

				attachDistance(assembly, partMatrix[i - 1], partMatrix[i], VxVector3(0, 0, 0), VxVector3(0, 0, 0), MaxDistance, threadStiffness, threadDamping);


			}
		}
	}

}
//void attachDistanceN(VxSmartInterface<Assembly> assembly, VxSmartInterface<Part> chaser, VxSmartInterface<Part> target, VxSmartInterface<Part>** partMatrix, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping)
//{
//}
