// Created by Eleonora Botta, September 10, 2016
// Modified by Eleonora Botta, September 12, 2016
// Modified by Eleonora Botta, October 5, 2016: possibility to give initial velocity to net nodes in ejection direction
// Modified by Eleonora Botta, October 14, 2016: contact materials assigned to parts, depending on their location
// Modified by Eleonora Botta, December 20, 2016: autocompute inertia properties
// Modified by Eleonora Botta, February 27, 2017: add possibility to model the net with cables
// Modified by Eleonora Botta, March 3, 2017: modified mass of net nodes: just mass of knots with cable-based model (netModel=2); mass of knots + lumped param mass with other models
// Modified by Eleonora Botta, March 10, 2017: consider mass of knots in finding 1st natural frequency
// Modified by Eleonora Botta, March 16, 2017: read some parameters for cables from netData.txt, if cable-based model is used (netModel=2)
//												modified numbering of netModel: 0 = LP, 1 = cable-based. bendingPresentInNet is always read to define if EI=0 or not

// Create a net made out of sphere nodes and cables between them simulated by distance or other joints in the xy plane, at z=Z, and with node(0)(0) in x=y=0. 

#include "MyLibraryNet/createNet.h"

#include "MyLibraryNet/addCornerMasses.h"
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/attachDistance.h"
#include "MyLibraryNet/attachPrismatic.h"
#include "MyLibraryNet/writeNaturalFrequencies.h"
//#include "MyLibraryNet/findNetFirstAxialNatFreq.h"
#include "MyLibraryNet/addCableForNetThread.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Prismatic.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Sphere.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxMath/Transformation.h>

#include <iostream>
#include <fstream>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;


float createNet(VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> **partMatrix, VxSmartInterface<DistanceJoint> allConstraintsVector[], VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[], VxReal totalMass, VxMaterial* myNetMaterial00, VxMaterial* myNetMaterial01, VxMaterial* myNetMaterial11, FILE* FileLog)
{
	// read constants
	std::stringstream constantsFileName;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for simulation
	std::stringstream simulationDataFileName;
	simulationDataFileName << pathData << "simulationData.txt";
	int netModel = readParameter<int>("netModel", simulationDataFileName);
	int CMsPresent = readParameter<int>("CMsPresent", simulationDataFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	VxReal NetSideLength = readParameter<VxReal>("NetSideLength", netDataFileName);
	VxReal lengthKnotThread = readParameter<VxReal>("lengthKnotThread", netDataFileName);
	VxReal Z = readParameter<VxReal>("Z", netDataFileName);
	VxReal compression = readParameter<VxReal>("compression", netDataFileName);
	VxReal netRadius = readParameter<VxReal>("netRadius", netDataFileName);
	VxReal netYoungModulus = readParameter<VxReal>("netYoungModulus", netDataFileName);
	VxReal netDensity = readParameter<VxReal>("netDensity", netDataFileName);
	VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
	VxReal csiBending = readParameter<VxReal>("csiBending", netDataFileName);
	VxReal netEjectionVelocity = readParameter<VxReal>("netEjectionVelocity", netDataFileName);
	int wayPointsNb, bendingPresentInNet, netCGtype;
	bendingPresentInNet = readParameter<int>("bendingPresentInNet", netDataFileName);
	if (netModel == 1)
	{
		wayPointsNb = readParameter<int>("wayPointsNb", netDataFileName);
		netCGtype = readParameter<int>("netCGtype", netDataFileName);
	}

	// define some useful quantities
	VxReal MeshLength = NetSideLength / (NetSize-1);
	VxReal stepX = MeshLength*compression;
	VxReal stepY = MeshLength*compression;
	int ucount = NetSize;
	int vcount = NetSize;
    assert(ucount < MAXGRIDSIZE && vcount < MAXGRIDSIZE);
	VxReal linearNetDensity = netDensity * PI * pow(netRadius,2.0);

	// read data for corner thread if CMs present (needed to compute mass of net corner masses)
	VxReal cornerThreadLength;
	VxReal cornerThreadRadius;
	if (CMsPresent==1)
	{
		std::stringstream CMsDataFileName;
		CMsDataFileName << pathData << "CMsData.txt";
		cornerThreadLength = readParameter<VxReal>("cornerThreadLength", CMsDataFileName);
		cornerThreadRadius = readParameter<VxReal>("cornerThreadRadius", CMsDataFileName);
		if ( cornerThreadLength==-1)
			cornerThreadLength = MeshLength*sqrt(2.0);
	}
	VxReal massHalfCornerThread = netDensity * cornerThreadLength * PI * pow(cornerThreadRadius,2) / 2;
	
	// find threads' stiffness
	VxReal threadStiffness = netYoungModulus * PI * pow(netRadius,2.00) / MeshLength;
	VxReal threadBendingStiffness = netYoungModulus * PI / 4.0 * pow(netRadius,4) / MeshLength; // EI/l
	VxReal threadTransverseStiffness = 3.0 * netYoungModulus * PI / 4.0 * pow(netRadius,4) / pow(MeshLength,3) ; // 3EI/l^3
	
	// find natural frequency for bending, found for thread 1 simply, and write it in a file 
	// for axial, call function that computes it from mass and stiffness matrices for the net
	VxReal massHalfThread = netDensity * MeshLength * PI * pow(netRadius,2) / 2;
	VxReal massKnot = netDensity * lengthKnotThread *PI * pow(netRadius,2);
	VxReal partMass = 4 * massHalfThread + massKnot;		// most of the net nodes' mass
	VxReal partInertia = partMass * pow(MeshLength,2);		// most of the net nodes' moment of inertia, cantilever model
	//double wn1Axial = findNetFirstAxialNatFreq(NetSize, CMsPresent, massHalfThread, massHalfCornerThread, massKnot, threadStiffness);
	double wn1Axial = 757.418;
	float wn1BendingRot = 1.25449;
	float wn1BendingTransl = 2.17284;
	writeNaturalFrequencies(pathData, wn1Axial, wn1BendingRot, wn1BendingTransl);
	

	// find threads' damping
	VxReal threadDamping = 2 * csiAxial / wn1Axial * threadStiffness;
	VxReal threadBendingDamping = 2 * csiBending / wn1BendingRot * threadBendingStiffness;  // da cambiare
	VxReal threadTransverseDamping = 2 * csiBending / wn1BendingTransl * threadTransverseStiffness; // da cambiare
		
	// initialize partMatrix
	VxMaterial* thisPartMaterial;
	VxVector3 lowerLeftCorner(0 , 0 , Z); // position of part [0][0]
    for (int i=0; i<ucount; i++)
    {
        for (int j=0; j<vcount; j++)
        {
			// part material, depending on part location
			thisPartMaterial = myNetMaterial11;
			if ( ( (i==0)&&(j==0) ) || ( (i==NetSize-1)&&(j==0) ) || ( (i==0)&&(j==NetSize-1) ) || ( (i==NetSize-1)&&(j==NetSize-1) )  )   // corner part 
				thisPartMaterial = myNetMaterial00;
			else if ( ( (i>0) && (i<NetSize-1) && (j==0) ) || ( (i>0) && (i<NetSize-1) && (j==NetSize-1) ) || ( (j>0) && (j<NetSize-1) && (i==0) ) || ( (j>0) && (j<NetSize-1) && (i==NetSize-1) )  )   // side part 
				thisPartMaterial = myNetMaterial01;

			// part mass, depending on part location
			if (netModel == 1)
				partMass = massKnot; 
			else
			{
				partMass = 4 * massHalfThread + massKnot; // central part 
				if ( ( (i==0)&&(j==0) ) || ( (i==NetSize-1)&&(j==0) ) || ( (i==0)&&(j==NetSize-1) ) || ( (i==NetSize-1)&&(j==NetSize-1) )  )   // corner part 
				{
					partMass = 2 * massHalfThread + massKnot; 
					if (CMsPresent==1)
						partMass = partMass + massHalfCornerThread;
				}
				else if ( ( (i>0) && (i<NetSize-1) && (j==0) ) || ( (i>0) && (i<NetSize-1) && (j==NetSize-1) ) || ( (j>0) && (j<NetSize-1) && (i==0) ) || ( (j>0) && (j<NetSize-1) && (i==NetSize-1) )  )   // side part 
				{
					partMass = 3 * massHalfThread + massKnot;
				}
			}
			//fprintf(FileLog, "\nThe mass of the sphere of index  %i,%i is %g kg \n", i,j,partMass);
			totalMass = totalMass + partMass;

			// part collision geometry
			VxSmartInterface<Sphere> sphere = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
			VxReal sphereRadius = pow((0.75 / PI * partMass / netDensity),(1/3.)); // x 10 for visualization only;
            sphere->parameterRadius = sphereRadius ;
			//fprintf(FileLog, "\nThe radius of the sphere of index  %i,%i is %g m \n", i,j,sphereRadius );
			
			// create part
			VxSmartInterface<Part> p = VxExtensionFactory::create(PartICD::kFactoryKey); 
			p->parameterMassPropertiesContainer.mass = partMass;
            VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
			cg = sphere;
			cg->parameterMaterial = thisPartMaterial;
            p->addCollisionGeometry(cg);
			p->autoComputeInertiaAndCOM();
            VxVector3 pos = lowerLeftCorner + VxVector3(stepX*i, stepY*j, 0);
			p->setLocalTransform(VxMath::Transformation::createTranslation(pos));
			p->setLinearVelocity( Vx::VxVector3( 0.0, 0.0, -netEjectionVelocity) );
			partMatrix[i][j] = p;
			//std::cout << "The position of the sphere of index " << i << "," << j << " is " << pos.x() << ", " << pos.y() << ", " << pos.z() << " m" << '\n'; 
            assembly->addPart(p);
           }        
    }

	// create constraints for net threads and store them in allConstraintsVector
	int counter = 0;
	for (int j=0; j<=NetSize-1; j++)
    {
        for (int i=0; i<=NetSize-1; i++)
		{
			if (i < NetSize-1)
			{
				if (netModel == 0) // no model of bending stiffness
				{
					if ( bendingPresentInNet == 0 )
						allConstraintsVector[counter] = attachDistance(assembly, partMatrix[i][j], partMatrix[i+1][j], MeshLength, threadStiffness, threadDamping);
					else if ( bendingPresentInNet == 1 )
					{
						allConstraintsVector[counter] = attachDistance(assembly, partMatrix[i][j], partMatrix[i+1][j], MeshLength, threadStiffness, threadDamping);
						allConstraintsVectorPrismatic[counter] = attachPrismatic(assembly, partMatrix[i][j], partMatrix[i+1][j], MeshLength, threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, threadTransverseStiffness, threadTransverseDamping);
					}
				}
				else if (netModel == 1)
				{
					// create cable for this thread 
					addCableForNetThread(mechanism, partMatrix[i][j], partMatrix[i+1][j], threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, MeshLength, netRadius, linearNetDensity, wayPointsNb, bendingPresentInNet, netCGtype);				
				}
				counter = counter+1;
			}
			if (j < NetSize-1)
			{
				if (netModel == 0)	// no model of bending stiffness
				{
					if ( bendingPresentInNet == 0 )
						allConstraintsVector[counter] = attachDistance(assembly, partMatrix[i][j], partMatrix[i][j+1], MeshLength, threadStiffness, threadDamping);
					else if ( bendingPresentInNet == 1 )
					{
						allConstraintsVector[counter] = attachDistance(assembly, partMatrix[i][j], partMatrix[i][j+1], MeshLength, threadStiffness, threadDamping);
						allConstraintsVectorPrismatic[counter] = attachPrismatic(assembly, partMatrix[i][j], partMatrix[i][j+1], MeshLength, threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, threadTransverseStiffness, threadTransverseDamping);
					}
				}
				else if (netModel == 1)
				{
					// crate cable for this corner thread 
					addCableForNetThread(mechanism, partMatrix[i][j], partMatrix[i][j+1], threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, MeshLength, netRadius, linearNetDensity, wayPointsNb, bendingPresentInNet, netCGtype);
				}
				counter = counter+1;
			}
		}
	}

	// print useful stuff in log file, to check
	fprintf(FileLog, "The net side length is %g m \n ", NetSideLength);
	fprintf(FileLog, "The mesh length is %g m \n ", MeshLength);
	fprintf(FileLog, "The net material Young's modulus is %g Pa \n", netYoungModulus);
	fprintf(FileLog, "The thread's radius is %g m \n ", netRadius);
	fprintf(FileLog, "\nThe threads' stiffness in axial direction is  %g N/m \n", threadStiffness);
	fprintf(FileLog, "\nThe threads' damping in axial direction  is  %g Ns/m \n", threadDamping);
	fprintf(FileLog, "\nThe threads' stiffness in bending (angular) is  %g Nm \n", threadBendingStiffness);
	fprintf(FileLog, "\nThe threads' damping in bending (angular) is  %g Nms \n", threadBendingDamping);
	fprintf(FileLog, "\nThe threads' stiffness in transverse direction is  %g N/m \n", threadTransverseStiffness);
	fprintf(FileLog, "\nThe threads' damping in transverse direction is  %g Ns/m \n", threadTransverseDamping);

	return totalMass;
}