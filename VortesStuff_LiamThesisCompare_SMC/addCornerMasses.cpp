// Created by Eleonora Botta, September 10, 2016
// Modified by Eleonora Botta, September 12, 2016
// Modified by Eleonora Botta, September 26, 2016: ejection velocity, given modulus and angle with ejection direction
// Modified by Eleonora Botta, October 14, 2016: contact materials assigned to parts
// Modified by Eleonora Botta, December 20, 2016: autocompute inertia properties
// Modified by Eleonora Botta, March 3, 2017: modified mass of corner masses: just mass of bullets with cable-based model (netModel=2); mass of bullets + lumped param mass with other models
// Modified by Eleonora Botta, March 16, 2017:  read some parameters for cables from netData.txt, if cable-based model is used (netModel=2)
//												modified numbering of netModel: 0 = LP, 1 = cable-based. bendingPresentInNet is always read to define if EI=0 or not

// To do: add material

#include "MyLibraryNet/addCornerMasses.h"
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/attachDistance.h"
#include "MyLibraryNet/attachPrismatic.h"
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
#include <cmath>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;


float addCornerMasses(VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[], VxReal totalMass, VxMaterial* myCmMaterial, FILE* FileLog)
{
	// read constants
	std::stringstream constantsFileName;
	std::ifstream inFile;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for simulation (model of net)
	std::stringstream simulationDataFileName;
	simulationDataFileName << pathData << "simulationData.txt";
	int netModel = readParameter<int>("netModel", simulationDataFileName);
	
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
	
	VxReal MeshLength = NetSideLength / (NetSize-1);
	VxReal linearNetDensity = netDensity * PI * pow(netRadius,2.0);

	int wayPointsNb, bendingPresentInNet, netCGtype;
	bendingPresentInNet = readParameter<int>("bendingPresentInNet", netDataFileName);
	if (netModel == 1)
	{
		wayPointsNb = readParameter<int>("wayPointsNb", netDataFileName);
		netCGtype = readParameter<int>("netCGtype", netDataFileName);
	}

	// read data for corner masses
	std::stringstream CMsDataFileName;
	CMsDataFileName << pathData << "CMsData.txt";
	VxReal BulletMass = readParameter<VxReal>("BulletMass", CMsDataFileName);
	VxReal alDensity = readParameter<VxReal>("alDensity", CMsDataFileName);
	VxReal cornerThreadLength = readParameter<VxReal>("cornerThreadLength", CMsDataFileName);
	VxReal cornerThreadRadius = readParameter<VxReal>("cornerThreadRadius", CMsDataFileName);
	VxReal ejectionVelocity = readParameter<VxReal>("ejectionVelocity", CMsDataFileName);
	VxReal shootingAngle = readParameter<VxReal>("shootingAngle", CMsDataFileName);
	if ( cornerThreadLength==-1)
		cornerThreadLength = MeshLength*sqrt(2.0);
		
	// find corner threads' stiffness
	VxReal CornerThreadStiffness = netYoungModulus * PI * pow(cornerThreadRadius,2.0) / cornerThreadLength;
	VxReal CornerThreadBendingStiffness = netYoungModulus * PI / 4.0 * pow(cornerThreadRadius,4) / cornerThreadLength; // EI/l corner threads
	VxReal CornerThreadTransverseStiffness = 3.0 * netYoungModulus * PI / 4.0 * pow(cornerThreadRadius,4) / pow(cornerThreadLength,3) ; // 3EI/l^3 corner threads
	
	// read natural frequencies
	std::stringstream natFreqFileName;
	natFreqFileName << pathData << "naturalFrequencies.txt";
	VxReal wn1Axial = readParameter<VxReal>("wn1Axial", natFreqFileName);
	VxReal wn1BendingRot = readParameter<VxReal>("wn1BendingRot", natFreqFileName);
	VxReal wn1BendingTransl = readParameter<VxReal>("wn1BendingTransl", natFreqFileName);

	// find corner threads' damping
	VxReal CornerThreadDamping = 2 * csiAxial / wn1Axial * CornerThreadStiffness;
	VxReal CornerThreadBendingDamping = 2 * csiBending / wn1BendingRot * CornerThreadBendingStiffness; // da cambiare
	VxReal CornerThreadTransverseDamping = 2 * csiBending / wn1BendingTransl * CornerThreadTransverseStiffness; // da cambiare
	
	// find corner masses' masses
	VxVector3 MassPosition(cornerThreadLength*sqrt(2.0)/2*compression, cornerThreadLength*sqrt(2.0)/2*compression, Z);
    VxReal massHalfCornerThread = netDensity * cornerThreadLength * PI * pow(cornerThreadRadius,2) / 2;
	VxReal CornerMass;
	if (netModel == 1)
		CornerMass = BulletMass;
	else 
		CornerMass = BulletMass + massHalfCornerThread;
	VxReal cornerMassRadius;
	for (int i=0; i<4; i++)
    {
		cornerMasses[i] = VxExtensionFactory::create(PartICD::kFactoryKey); 
		totalMass = totalMass + CornerMass;
		cornerMasses[i]->parameterMassPropertiesContainer.mass = CornerMass;
		VxSmartInterface<Sphere> sphere = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
		cornerMassRadius = pow(0.75 * CornerMass / PI / alDensity , 1/3.); // x 10 for visualization only; 
		sphere->parameterRadius = cornerMassRadius;
        VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
		cg = sphere;
		cg->parameterMaterial = myCmMaterial;
        cornerMasses[i]->addCollisionGeometry(cg);
        cornerMasses[i]->autoComputeInertiaAndCOM();

		fprintf(FileLog, "\nThe mass of the corner mass of index  %i is %g kg \n", i, CornerMass);
		fprintf(FileLog, "\nThe radius of the corner mass of index  %i is %g m \n", i, cornerMassRadius);
     }   
	
	// set corner masses position
	cornerMasses[0]->setLocalTransform(VxMath::Transformation::createTranslation(Vx::VxVector3(-MassPosition[0], -MassPosition[1], MassPosition[2])));
	cornerMasses[1]->setLocalTransform(VxMath::Transformation::createTranslation(Vx::VxVector3(NetSideLength*compression + MassPosition[0], -MassPosition[1], MassPosition[2])));
	cornerMasses[2]->setLocalTransform(VxMath::Transformation::createTranslation(Vx::VxVector3(-MassPosition[0], NetSideLength*compression + MassPosition[1], MassPosition[2])));
	cornerMasses[3]->setLocalTransform(VxMath::Transformation::createTranslation(Vx::VxVector3(NetSideLength*compression + MassPosition[0], NetSideLength*compression + MassPosition[1], MassPosition[2])));
	
	// find x,y,z components of ejection velocity, given the ejection velocity modulus and the angle with the ejection direction (z)
	VxReal shootingAngleRad = shootingAngle * PI / 180.0;
	VxReal ejectionVelocity_X = ejectionVelocity * sin(shootingAngleRad) / pow(2.0, 0.5);
	VxReal ejectionVelocity_Y = ejectionVelocity_X;
	VxReal ejectionVelocity_Z = ejectionVelocity * cos(shootingAngleRad);
	
	// set corner masses velocity
	cornerMasses[0]->setLinearVelocity( Vx::VxVector3(-ejectionVelocity_X, -ejectionVelocity_Y, -ejectionVelocity_Z) );
    cornerMasses[1]->setLinearVelocity( Vx::VxVector3( ejectionVelocity_X, -ejectionVelocity_Y, -ejectionVelocity_Z) );
    cornerMasses[2]->setLinearVelocity( Vx::VxVector3(-ejectionVelocity_X,  ejectionVelocity_Y, -ejectionVelocity_Z) );
    cornerMasses[3]->setLinearVelocity( Vx::VxVector3( ejectionVelocity_X,  ejectionVelocity_Y, -ejectionVelocity_Z) );

	// add corner masses to assembly
	for (int i=0; i<4; i++)
    {
		//std::cout << "The position of the corner mass of index " << i << " is " << cornerMasses[i]->getPosition().x() << ", " << cornerMasses[i]->getPosition().y() << ", " << cornerMasses[i]->getPosition().z() << " m" << '\n'; 
        assembly->addPart(cornerMasses[i]);
    }
	
	// create constraints for corner threads and store them in allConstraintsVector (appended after net's threads)
	if (netModel == 0)		// no model of bending stiffness
	{
		if ( bendingPresentInNet == 0 )
		{
			allConstraintsVector[2*NetSize*(NetSize-1) ] = attachDistance( assembly, cornerMasses[0], partMatrix[0][0], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping);
			allConstraintsVector[2*NetSize*(NetSize-1) + 1 ] =  attachDistance( assembly, cornerMasses[1], partMatrix[NetSize-1][0], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping); //NetSize-1 is the index of the last part in that row
			allConstraintsVector[2*NetSize*(NetSize-1) + 2 ] = attachDistance( assembly, cornerMasses[2], partMatrix[0][NetSize-1], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping);
			allConstraintsVector[2*NetSize*(NetSize-1) + 3 ] = attachDistance( assembly, cornerMasses[3], partMatrix[NetSize-1][NetSize-1], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping);
		}
		else if ( bendingPresentInNet == 1 )
		{
			allConstraintsVector[2*NetSize*(NetSize-1) ] = attachDistance( assembly, cornerMasses[0], partMatrix[0][0], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping);
			allConstraintsVector[2*NetSize*(NetSize-1) + 1 ] =  attachDistance( assembly, cornerMasses[1], partMatrix[NetSize-1][0], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping); //NetSize-1 is the index of the last part in that row
			allConstraintsVector[2*NetSize*(NetSize-1) + 2 ] = attachDistance( assembly, cornerMasses[2], partMatrix[0][NetSize-1], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping);
			allConstraintsVector[2*NetSize*(NetSize-1) + 3 ] = attachDistance( assembly, cornerMasses[3], partMatrix[NetSize-1][NetSize-1], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping);
			allConstraintsVectorPrismatic[2*NetSize*(NetSize-1) ] = attachPrismatic( assembly, cornerMasses[0], partMatrix[0][0], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, CornerThreadTransverseStiffness, CornerThreadTransverseDamping);
			allConstraintsVectorPrismatic[2*NetSize*(NetSize-1) + 1 ] =  attachPrismatic( assembly, cornerMasses[1], partMatrix[NetSize-1][0], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, CornerThreadTransverseStiffness, CornerThreadTransverseDamping);
			allConstraintsVectorPrismatic[2*NetSize*(NetSize-1) + 2 ] = attachPrismatic( assembly, cornerMasses[2], partMatrix[0][NetSize-1], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, CornerThreadTransverseStiffness, CornerThreadTransverseDamping);
			allConstraintsVectorPrismatic[2*NetSize*(NetSize-1) + 3 ] = attachPrismatic( assembly, cornerMasses[3], partMatrix[NetSize-1][NetSize-1], cornerThreadLength, CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, CornerThreadTransverseStiffness, CornerThreadTransverseDamping);
		}
	}
	else if (netModel == 1)		
	{
		addCableForNetThread(mechanism, cornerMasses[0], partMatrix[0][0], CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, cornerThreadLength, cornerThreadRadius, linearNetDensity, wayPointsNb, bendingPresentInNet, netCGtype);
		addCableForNetThread(mechanism, cornerMasses[1], partMatrix[NetSize-1][0], CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, cornerThreadLength, cornerThreadRadius, linearNetDensity, wayPointsNb, bendingPresentInNet, netCGtype);
		addCableForNetThread(mechanism, cornerMasses[2], partMatrix[0][NetSize-1], CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, cornerThreadLength, cornerThreadRadius, linearNetDensity, wayPointsNb, bendingPresentInNet, netCGtype);
		addCableForNetThread(mechanism, cornerMasses[3], partMatrix[NetSize-1][NetSize-1], CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, cornerThreadLength, cornerThreadRadius, linearNetDensity, wayPointsNb, bendingPresentInNet, netCGtype);
	}

	// print stuff just to check
	fprintf(FileLog, "\nThe length of the corner threads  is %g m \n", cornerThreadLength);
	fprintf(FileLog, "\nThe stiffness of the corner threads  is %g N/m \n", CornerThreadStiffness);
	fprintf(FileLog, "\nThe damping of the corner threads  is %g kg Ns/m", CornerThreadDamping);
	fprintf(FileLog, "Each bullet's mass is %g kg \n ", BulletMass );
	fprintf(FileLog, "The bullets material density is %g kg/m3 \n ", alDensity);
	fprintf(FileLog, "The corner thread lenght is %g m \n ", cornerThreadLength);
	fprintf(FileLog, "\nThe corner threads' stiffness in axial direction is  %g N/m \n", CornerThreadStiffness);
	fprintf(FileLog, "\nThe corner threads' damping in axial direction  is  %g Ns/m \n\n", CornerThreadDamping);
	fprintf(FileLog, "\nThe corner threads' stiffness in bending (angular) is  %g Nm \n", CornerThreadBendingStiffness);
	fprintf(FileLog, "\nThe corner threads' damping in bending (angular) is  %g Nms \n\n", CornerThreadBendingDamping);
	fprintf(FileLog, "\nThe corner threads' stiffness in transverse direction is  %g N/m \n", CornerThreadTransverseStiffness);
	fprintf(FileLog, "\nThe corner threads' damping in transverse direction is  %g Ns/m \n\n", CornerThreadTransverseDamping);

	return totalMass;
}
