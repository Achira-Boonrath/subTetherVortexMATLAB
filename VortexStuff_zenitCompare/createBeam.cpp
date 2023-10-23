// Created by Eleonora Botta, September 10, 2016
// Modified by Eleonora Botta, September 12, 2016

// Create a cantilever beam made out of sphere nodes and cables between them simulated by distance or other joints in the xy plane, at z=Z, and centered at x,y=0. 

#include "MyLibraryNet/createBeam.h"

#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/attachDistance.h"
#include "MyLibraryNet/attachPrismatic.h"
#include "MyLibraryNet/writeNaturalFrequencies.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Prismatic.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Sphere.h>
#include <VxDynamics/Box.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxMath/Transformation.h>

#include <iostream>
#include <fstream>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;


float createBeam(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> partMatrix[], VxSmartInterface<DistanceJoint> allConstraintsVector[], VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[], VxReal totalMass, FILE* FileLog)
{
	// read constants
	std::stringstream constantsFileName;
	std::ifstream inFile;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for simulation (netModel)
	std::stringstream simulationDataFileName;
	simulationDataFileName << pathData << "simulationData.txt";
	int netModel = readParameter<int>("netModel", simulationDataFileName);
	int CMsPresent = readParameter<int>("CMsPresent", simulationDataFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	VxReal NetSideLength = readParameter<VxReal>("NetSideLength", netDataFileName);
	VxReal Z = readParameter<VxReal>("Z", netDataFileName);
	VxReal compression = readParameter<VxReal>("compression", netDataFileName);
	VxReal netRadius = readParameter<VxReal>("netRadius", netDataFileName);
	VxReal netYoungModulus = readParameter<VxReal>("netYoungModulus", netDataFileName);
	VxReal netDensity = readParameter<VxReal>("netDensity", netDataFileName);
	VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
	VxReal csiBending = readParameter<VxReal>("csiBending", netDataFileName);

	// define some useful quantities
	VxReal MeshLength = NetSideLength / (NetSize-1);
	VxReal stepX = MeshLength*compression;
	VxReal stepY = MeshLength*compression;
	int ucount = NetSize;
	int vcount = NetSize;
    assert(ucount < MAXGRIDSIZE && vcount < MAXGRIDSIZE);

	// read data for corner thread if CMs present (needed to compute mass of net corner masses)
	VxReal cornerThreadLength;
	VxReal cornerThreadRadius;
	if (CMsPresent==1)
	{
		std::cout << "Corner masses. Does not make sense for cantilever beam." << std::endl;
		std::cin;
	}
	VxReal massHalfCornerThread = netDensity * cornerThreadLength * PI * pow(cornerThreadRadius,2) / 2;
	
	// find threads' stiffness
	VxReal threadStiffness = netYoungModulus * PI * pow(netRadius,2.00) / MeshLength;
	VxReal threadBendingStiffness = netYoungModulus * PI / 4.0 * pow(netRadius,4) / MeshLength; // EI/l
	VxReal threadTransverseStiffness = 3.0 * netYoungModulus * PI / 4.0 * pow(netRadius,4) / pow(MeshLength,3) ; // 3EI/l^3
	
	// find natural frequency for bending, found for thread 1 simply, and write it in a file 
	// for axial, call function that computes it from mass and stiffness matrices for the net
	VxReal massHalfThread = netDensity * MeshLength * PI * pow(netRadius,2) / 2;
	double wn1Axial;
	float wn1BendingRot;
	float wn1BendingTransl;
	if (NetSize == 2)
	{
		wn1Axial = 20367.0030886;	
		wn1BendingRot =  249.443825; 
		wn1BendingTransl =  249.443825; 
	}
	else if (NetSize == 3)
	{
		wn1Axial = 28803.2919929238;
		wn1BendingRot = 705.5336829505;
		wn1BendingTransl =  705.5336829505;
	}
	else if (NetSize == 10)
	{
		wn1Axial = 31830.23673799397;
		wn1BendingRot = 14287.0570797487;
		wn1BendingTransl =  14287.0570797487;
	}
	writeNaturalFrequencies(pathData, wn1Axial, wn1BendingRot, wn1BendingTransl);
	
	// find threads' damping
	VxReal threadDamping = 2 * csiAxial / wn1Axial * threadStiffness;
	VxReal threadBendingDamping = 2 * csiBending / wn1BendingRot * threadBendingStiffness;  // da cambiare
	VxReal threadTransverseDamping = 2 * csiBending / wn1BendingTransl * threadTransverseStiffness; // da cambiare
		
	// initialize partMatrix
	VxVector3 lowerLeftCorner(0 , 0 , Z); // position of part [0][0]

        for (int j=0; j<vcount; j++)
        {
			VxReal partMass = 2 * massHalfThread;
			// part mass
			if (  (j==0)  || (j==NetSize-1) )  
				partMass = 1 * netDensity * MeshLength * PI * pow(netRadius,2) / 2 ;
			fprintf(FileLog, "\nThe mass of the sphere of index  %i is %f kg \n", j,partMass);
			totalMass = totalMass + partMass;
			// part collision geometry
			VxSmartInterface<Sphere> sphere = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
			VxReal sphereRadius = pow((0.75 / PI * partMass / netDensity),(1/3.)); // x 10 for visualization only;
            sphere->parameterRadius = sphereRadius ;
			fprintf(FileLog, "\nThe radius of the sphere of index  %i is %f m \n", j,sphereRadius );
			// create part
			VxSmartInterface<Part> p = VxExtensionFactory::create(PartICD::kFactoryKey); 
			p->parameterMassPropertiesContainer.mass = partMass;
            /*VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
			cg = sphere;*/
			VxSmartInterface<Box> box = VxExtensionFactory::create(VxDynamics::Box::kFactoryKey); 
			p->parameterMassPropertiesContainer.mass = partMass;
			box->parameterDimension = VxVector3( 2*sphereRadius,2*sphereRadius,2*sphereRadius);
			VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Box::kFactoryKey); 
            cg = box;
			p->addCollisionGeometry(cg);
            VxVector3 pos = lowerLeftCorner + VxVector3(stepX*j, 0, 0);
            /*VxPart* part = p->getVxPart();
			part->setPosition(pos);*/
			p->setLocalTransform(VxMath::Transformation::createTranslation(pos));
			partMatrix[j] = p;
			//std::cout << "The position of the sphere of index " << i << "," << j << " is " << pos.x() << ", " << pos.y() << ", " << pos.z() << " m" << '\n'; 
            if (j==0)  p->getVxPart()->freeze(true);
						//p->setControl(VxPart::kControlStatic);
			if (j==NetSize-1)
			{
				// choose value for tip force
				//VxReal3 externalForceTip = {0,0,-40};
				VxReal3 externalForceTip = {0,0,-2620};
				p->getVxPart()->addForce(externalForceTip);
			}
			assembly->addPart(p);     
			// create constraints for net threads and store them in allConstraintsVector
			if (j>0) 
			{
				//allConstraintsVector[j-1] = attachDistance(assembly, partMatrix[j-1], p, MeshLength, threadStiffness, threadDamping);			
				allConstraintsVectorPrismatic[j-1] = attachPrismatic(assembly, partMatrix[j-1], p, MeshLength, threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, threadTransverseStiffness, threadTransverseDamping);			
			}  
		}


	// print useful stuff in log file, to check
	fprintf(FileLog, "The net side lenght is %f m \n ", NetSideLength);
	fprintf(FileLog, "The mesh lenght is %f m \n ", MeshLength);
	fprintf(FileLog, "The net material Young's modulus is %f Pa \n", netYoungModulus);
	fprintf(FileLog, "The thread's radius is %f m \n ", netRadius);
	fprintf(FileLog, "\nThe threads' stiffness in axial direction is  %f N/m \n", threadStiffness);
	fprintf(FileLog, "\nThe threads' damping in axial direction  is  %f Ns/m \n", threadDamping);
	fprintf(FileLog, "\nThe threads' stiffness in bending (angular) is  %f Nm \n", threadBendingStiffness);
	fprintf(FileLog, "\nThe threads' damping in bending (angular) is  %f Nms \n", threadBendingDamping);
	fprintf(FileLog, "\nThe threads' stiffness in transverse direction is  %f N/m \n", threadTransverseStiffness);
	fprintf(FileLog, "\nThe threads' damping in transverse direction is  %f Ns/m \n", threadTransverseDamping);

	return totalMass;
}