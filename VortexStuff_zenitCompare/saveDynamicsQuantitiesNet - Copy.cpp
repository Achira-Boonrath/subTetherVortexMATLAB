// Created by Eleonora Botta, September 6, 2016
// Modified by Eleonora Botta, September 12, 2016
// Modified by Eleonora Botta, September 23, 2016: pass partMatrix as VxSmartInterface<Part> **partMatrix
// Modified by Eleonora Botta, March 16, 2017: modified numbering of netModel: 0 = LP, 1 = cable-based. 
// Modified by Eleonora Botta, April 1, 2017: add function to save all masses of the net (and CMs) in a file.
// Modified by Rachael Gold, August 5, 2020: Modified to include inner nodes model (netModel = 5). Note that inner nodes model uses CornerMassMatrix rather than cornerMasses  

#include "MyLibraryNet/saveDynamicsQuantitiesNet.h"

#include <iostream>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>

#define MAXGRIDSIZE 301

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;


//void saveDynamicsQuantitiesNet(int netModel, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], FILE* outX, FILE* outY, FILE* outZ, FILE* outVX, FILE* outVY, FILE* outVZ, FILE* outF, int NetSize, int CMsPresent, int NI,)
void saveDynamicsQuantitiesNet(int netModel, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> **InnerNodeMatrix, VxSmartInterface<Part> **CornerMassMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], FILE* outX, FILE* outY, FILE* outZ, FILE* outVX, FILE* outVY, FILE* outVZ, FILE* outXInner, FILE* outYInner, FILE* outZInner, FILE* outVXInner, FILE* outVYInner, FILE* outVZInner,  FILE* outF, int NetSize, int CMsPresent, int NI, int N_CornerThread, int numNodes, int NumNodesToAdd)
 {
	//if (netModel == 10)
	//{
	//	NI = 1;
	//}

	int N = NetSize + (NetSize - 1)*NI;
	int threadCount = 2 * (NetSize * (NetSize - 1)); //total number of threads 
	int NIMax = NumNodesToAdd; 

	 // save dynamics quantities for net
	if (netModel != 5)
	{
		for (int j = 0; j < NetSize; j++)
		{
			for (int i = 0; i < NetSize; i++)
			{

				VxVector3 pos = partMatrix[i][j]->getVxPart()->getPosition();
				VxVector3 vel = partMatrix[i][j]->getVxPart()->getLinearVelocity();
				fprintf(outX, "%g ", pos.x()); // out x to the file
				fprintf(outY, "%g ", pos.y());
				fprintf(outZ, "%g ", pos.z());
				fprintf(outVX, "%g ", vel.x());
				fprintf(outVY, "%g ", vel.y());
				fprintf(outVZ, "%g ", vel.z());

			}
		}

		if (netModel == 10)
		{ 
			for (int r = 0; r < threadCount; r++)
			{
				for (int q = 0; q < NIMax; q++)
				{
					if (InnerNodeMatrix[r][q] != nullptr)
					{
						VxVector3 pos = InnerNodeMatrix[r][q]->getVxPart()->getPosition();
						VxVector3 vel = InnerNodeMatrix[r][q]->getVxPart()->getLinearVelocity();
						fprintf(outXInner, "%g ", pos.x()); // out x to the file
						fprintf(outYInner, "%g ", pos.y());
						fprintf(outZInner, "%g ", pos.z());
						fprintf(outVXInner, "%g ", vel.x());
						fprintf(outVYInner, "%g ", vel.y());
						fprintf(outVZInner, "%g ", vel.z());
					}

					else
					{
						fprintf(outXInner, "NaN "); //if the inner node in question does not exist, output NaN
						fprintf(outYInner, "NaN ");
						fprintf(outZInner, "NaN ");
						fprintf(outVXInner, "NaN ");
						fprintf(outVYInner, "NaN ");
						fprintf(outVZInner, "NaN ");
					
					}
				}
			}

			for (int j = 0; j < NetSize; j++)
			{
				for (int i = 0; i < NetSize; i++)
				{

					VxVector3 pos = partMatrix[i][j]->getVxPart()->getPosition();
					VxVector3 vel = partMatrix[i][j]->getVxPart()->getLinearVelocity();
					fprintf(outXInner, "%g ", pos.x()); // out x to the file
					fprintf(outYInner, "%g ", pos.y());
					fprintf(outZInner, "%g ", pos.z());
					fprintf(outVXInner, "%g ", vel.x());
					fprintf(outVYInner, "%g ", vel.y());
					fprintf(outVZInner, "%g ", vel.z());

					VxReal CMtension_count = partMatrix[i][j]->getVxPart()->getConstraintCount();
					VxReal CMtension = partMatrix[i][j]->getVxPart()->getConstraint(0)->getConstraintEquationForce(0);

					if (CMtension * CMtension > 0.0000000000001)
					{
						//std::cout << CMtension << std::endl;
					}

				}
			}
			if (CMsPresent == 1)
			{
				for (int j = 0; j < 4; j++)
				{
					VxVector3 pos = cornerMasses[j]->getVxPart()->getPosition();
					VxVector3 vel = cornerMasses[j]->getVxPart()->getLinearVelocity();
					fprintf(outXInner, "%g ", pos.x()); // out x to the file
					fprintf(outYInner, "%g ", pos.y());
					fprintf(outZInner, "%g ", pos.z());
					fprintf(outVXInner, "%g ", vel.x());
					fprintf(outVYInner, "%g ", vel.y());
					fprintf(outVZInner, "%g ", vel.z());
				}
			}

		}

	}
	else
	{
		for (int j = 0; j < N; j++)
		{
			for (int i = 0; i < N; i++)
			{
				if ((remainder(i, (NI + 1)) != 0) && (remainder(j, (NI + 1)) == 0) || (remainder(i, (NI + 1)) == 0) && (remainder(j, (NI + 1)) != 0) || (remainder(i, (NI + 1)) == 0) && (remainder(j, (NI + 1)) == 0))
				{
					VxVector3 pos = partMatrix[i][j]->getVxPart()->getPosition();
					VxVector3 vel = partMatrix[i][j]->getVxPart()->getLinearVelocity();
					fprintf(outX, "%g ", pos.x()); // out x to the file
					fprintf(outY, "%g ", pos.y());
					fprintf(outZ, "%g ", pos.z());
					fprintf(outVX, "%g ", vel.x());
					fprintf(outVY, "%g ", vel.y());
					fprintf(outVZ, "%g ", vel.z());


					
				}
			}
		}
	}

	 // save dynamics quantities for corner masses - if present
	 if (CMsPresent==1)
	 {
		 if (netModel != 5 && netModel != 10)
		 {
			 for (int j = 0; j < 4; j++)
			 {
				 VxVector3 pos = cornerMasses[j]->getVxPart()->getPosition();
				 VxVector3 vel = cornerMasses[j]->getVxPart()->getLinearVelocity();
				 fprintf(outX, "%g ", pos.x()); // out x to the file
				 fprintf(outY, "%g ", pos.y());
				 fprintf(outZ, "%g ", pos.z());
				 fprintf(outVX, "%g ", vel.x());
				 fprintf(outVY, "%g ", vel.y());
				 fprintf(outVZ, "%g ", vel.z());
			 }
		 }
		 else if (netModel == 10)
		 {
			 for (int j = 0; j < 4; j++)
			 {
				 VxVector3 pos = cornerMasses[j]->getVxPart()->getPosition();
				 VxVector3 vel = cornerMasses[j]->getVxPart()->getLinearVelocity();
				 fprintf(outX, "%g ", pos.x()); // out x to the file
				 fprintf(outY, "%g ", pos.y());
				 fprintf(outZ, "%g ", pos.z());
				 fprintf(outVX, "%g ", vel.x());
				 fprintf(outVY, "%g ", vel.y());
				 fprintf(outVZ, "%g ", vel.z());

				// VxReal CMtension_count = cornerMasses[j]->getVxPart()->getConstraintCount();
				// VxReal CMtension = cornerMasses[j]->getVxPart()->getConstraint(0)->getConstraintEquationForce(1);
				// //VxReal CMtension = cornerMasses[j]->getVxPart()->getMass();

				// if (CMtension * CMtension > 0.0000000000001)
				// {
				//	 //std::cout << CMtension << std::endl;
				// }

				// VxConstraint* partCm = cornerMasses[j]->getVxPart()->getConstraint(0);
				// VxReal TensionForce = partCm->getConstraintEquationForce(0);
				////fprintf(outF, "%1.15f ", TensionForce);
				//if ((TensionForce > 0.0001) || (TensionForce < -0.0001))
				//{
 			//		//std::cout << TensionForce << std::endl;
				// }

				////std::cout << partCm->getConstraintEquationForce(0) << std::endl;
				////std::cout << partCm->getConstraintEquationForce(1) << std::endl;
				////std::cout << partCm->getConstraintEquationForce(2) << std::endl;
				////std::cout << partCm->getConstraintEquationForce(3) << std::endl;

			 }
		 }
		 else
			 for (int i = 0; i < 4; i++)
			 {
				 for (int j = 0; j < N_CornerThread; j++)
				 {
						VxVector3 pos = CornerMassMatrix[i][j]->getVxPart()->getPosition();
						VxVector3 vel = CornerMassMatrix[i][j]->getVxPart()->getLinearVelocity();
						fprintf(outX, "%g ", pos.x()); // out x to the file
						fprintf(outY, "%g ", pos.y());
						fprintf(outZ, "%g ", pos.z());
						fprintf(outVX, "%g ", vel.x());
						fprintf(outVY, "%g ", vel.y());
						fprintf(outVZ, "%g ", vel.z());
					 
				 }
			 }
	 }

	// save tensions in threads
	/*VxReal threadsNumber;
	if (CMsPresent==0)
		threadsNumber = 2*NetSize*(NetSize-1);
	else if (CMsPresent==1)
		threadsNumber = 2*NetSize*(NetSize-1)+4;
	*/

	//if (netModel == 0)
	//{
	//	for (int k=0; k<threadsNumber; k++) // save tension for attachDistance
	//	{
	//		VxReal TensionForce = allConstraintsVector[k]->getVxConstraint()->getConstraintEquationForce(0);
	//		fprintf(outF, "%1.15f ", TensionForce ); 
	//	}
	//}

	fprintf(outX, "\n"); // new line to the file
	fprintf(outY, "\n");
	fprintf(outZ, "\n");
	fprintf(outVX, "\n"); 
	fprintf(outVY, "\n");
	fprintf(outVZ, "\n");
	fprintf(outXInner, "\n");
	fprintf(outYInner, "\n");
	fprintf(outZInner, "\n");
	fprintf(outVXInner, "\n");
	fprintf(outVYInner, "\n");
	fprintf(outVZInner, "\n");

	//fprintf(outF, "\n");
 }
 

void saveMassNet(VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part>** InnerNodeMatrix, VxSmartInterface<Part> cornerMasses[], FILE* outMass, int NetSize, int CMsPresent, int netModel, int NI, int NumNodesToAdd)
{
	int N = NetSize + (NetSize - 1) * NI;
	int threadCount = 2 * (NetSize * (NetSize - 1)); //total number of threads 
	int NIMax = NumNodesToAdd; //CW - Arbitrary for now

	if (netModel != 10)
	{
		for (int j = 0; j < NetSize; j++)
		{
			for (int i = 0; i < NetSize; i++)
			{
				VxReal partMass = partMatrix[i][j]->getVxPart()->getMass();
				fprintf(outMass, "%.15f ", partMass);
			}
		}
		if (CMsPresent == 1)
		{
			for (int j = 0; j < 4; j++)
			{
				VxReal partMass = cornerMasses[j]->getVxPart()->getMass();
				fprintf(outMass, "%.15f ", partMass);
			}
		}
	}
	else if (netModel == 10)
	{
		for (int r = 0; r < threadCount; r++)
		{
			for (int q = 0; q < NIMax; q++)
			{
				if (InnerNodeMatrix[r][q] != nullptr)
				{
					double partMass = InnerNodeMatrix[r][q]->getVxPart()->getMass();
					fprintf(outMass, "%.15f ", partMass); // out x to the file
				}

				else
				{
					fprintf(outMass, "NaN "); //if the inner node in question does not exist, output NaN

				}
			}
		}

		for (int j = 0; j < NetSize; j++)
		{
			for (int i = 0; i < NetSize; i++)
			{

				double partMass = partMatrix[i][j]->getVxPart()->getMass();
				fprintf(outMass, "%.15f ", partMass); // out x to the file


			}
		}
		if (CMsPresent == 1)
		{
			for (int j = 0; j < 4; j++)
			{
				double partMass = cornerMasses[j]->getVxPart()->getMass();
				fprintf(outMass, "%.15f ", partMass); // out x to the file

			}
		}
	}

	fprintf(outMass, "\n");
}
