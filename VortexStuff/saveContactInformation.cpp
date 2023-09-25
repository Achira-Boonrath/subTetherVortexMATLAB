// Retrieve and save contact information for all the contacts at a certain time 
// Created by Eleonora Botta, October 27, 2016

#include "MyLibraryNet/saveContactInformation.h"

#include <iostream>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include "Vx/VxDynamicsResponseInput.h"
#include <Vx/VxContactMaterial.h>
#include <Vx/VxUniverse.h>
#include <VxDynamics/Part.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

void saveContactInformation(VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], float currentTime, int NetSize, int CMsPresent, FILE* outContact)
{
	for (int i=0; i<NetSize; i++)
	{
		for (int j=0; j<NetSize; j++)
		{
			//if ( (i==0)&&(j==18) )
			//{
			//VxVector3 posP = partMatrix[i][j]->getVxPart()->getPosition();
			//VxVector3 velP = partMatrix[i][j]->getVxPart()->getLinearVelocity();
			//VxVector3 posP2 = partMatrix[1][19]->getVxPart()->getPosition();
			//VxVector3 velP2 = partMatrix[1][19]->getVxPart()->getLinearVelocity();
			//fprintf(outContact, "%g \t ( %g , %g , %g ) \t (%g , %g , %g )  \t ( %g , %g , %g ) \t (%g , %g , %g )\t", currentTime, posP.x(),posP.y(),posP.z(), velP.x(),velP.y(),velP.z() , posP2.x(),posP2.y(),posP2.z(), velP2.x(),velP2.y(),velP2.z() );  
			for( Vx::VxPart::DynamicsContactIterator it = partMatrix[i][j]->getVxPart()->dynamicsContactBegin(); it != partMatrix[i][j]->getVxPart()->dynamicsContactEnd(); ++it)
			{
				VxVector3 force;
				VxReal3 normal;
				VxReal3 primaryDir;
				VxReal3 secondaryDir;
				VxReal slidingVel1Dir;
				VxReal slidingVel2Dir;
				VxDynamicsContact* thisContact = *it; 
				//std::cout << "Is contact matched?  " << thisContact->getMatched() << std::endl;
				thisContact->getForce(1,force);
				thisContact->getNormal(normal);
				thisContact->getPrimaryDirection(primaryDir);
				thisContact->getSecondaryDirection(secondaryDir);
				thisContact->getSlidingVelocity(slidingVel1Dir,slidingVel2Dir);
				VxContactMaterial* thisContactMaterialProps = thisContact->getContactMaterial();
				
				// find the other part
				Vx::VxPart* p1;
				Vx::VxPart* p2;
				thisContact->getPartPair(&p1,&p2);
				int idx2 = -1000;
				int idy2 = -1000;
				for (int p=i; p<NetSize; p++)
				{
					for (int q=0; q<NetSize; q++)
					{
						if ( p1 == partMatrix[p][q]->getVxPart() )
						{
							idx2 = p;
							idy2 = q;
							if ( (p!=i)||(q!=j) ) 
								fprintf(outContact, "%g \t (%d,%d) \t (%d,%d) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t %g  \t %g  \t %g \t %g \t %g  \t %g  \t %g  \t %g \n", currentTime, i,j, idx2,idy2, force.x(),force.y(),force.z(), normal[0],normal[1],normal[2], primaryDir[0],primaryDir[1],primaryDir[2], secondaryDir[0],secondaryDir[1],secondaryDir[2], thisContact->getPenetration(), slidingVel1Dir, slidingVel2Dir, thisContactMaterialProps->getCompliance() ,thisContactMaterialProps->getDamping(), thisContactMaterialProps->getAdhesiveForce(), thisContactMaterialProps->getRestitution(), thisContactMaterialProps->getRestitutionThreshold() );  
							break;
						} 
					}
				}
				if ( (idx2==-1000) && (CMsPresent==1) )
					for (int p=0; p<4; p++)
						if ( p1 == cornerMasses[p]->getVxPart() )
						{
							idx2 = p;
							fprintf(outContact, "%g \t (%d,%d) \t (CM %d) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t %g  \t %g  \t %g \t %g \t %g  \t %g  \t %g  \t %g \n", currentTime, i,j, idx2, force.x(),force.y(),force.z(), normal[0],normal[1],normal[2], primaryDir[0],primaryDir[1],primaryDir[2], secondaryDir[0],secondaryDir[1],secondaryDir[2], thisContact->getPenetration(), slidingVel1Dir, slidingVel2Dir, thisContactMaterialProps->getCompliance() ,thisContactMaterialProps->getDamping(), thisContactMaterialProps->getAdhesiveForce(), thisContactMaterialProps->getRestitution(), thisContactMaterialProps->getRestitutionThreshold() );  
							break;
						} 
				if (idx2==-1000) 
					fprintf(outContact, "%g \t (%d,%d) \t (debris) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t %g  \t %g  \t %g \t %g \t %g  \t %g  \t %g  \t %g \n", currentTime, i,j, force.x(),force.y(),force.z(), normal[0],normal[1],normal[2], primaryDir[0],primaryDir[1],primaryDir[2], secondaryDir[0],secondaryDir[1],secondaryDir[2], thisContact->getPenetration(), slidingVel1Dir, slidingVel2Dir, thisContactMaterialProps->getCompliance() ,thisContactMaterialProps->getDamping(), thisContactMaterialProps->getAdhesiveForce(), thisContactMaterialProps->getRestitution(), thisContactMaterialProps->getRestitutionThreshold() );  
							
			}
							
		}
	}

	if (CMsPresent==1)
	{
		for (int j=0; j<4; j++)
		{
			for( Vx::VxPart::DynamicsContactIterator it = cornerMasses[j]->getVxPart()->dynamicsContactBegin(); it != cornerMasses[j]->getVxPart()->dynamicsContactEnd(); ++it)
			{
				VxVector3 force;
				VxReal3 normal;
				VxReal3 primaryDir;
				VxReal3 secondaryDir;
				VxReal slidingVel1Dir;
				VxReal slidingVel2Dir;
				VxDynamicsContact* thisContact = *it; 
				//std::cout << "Is contact matched?  " << thisContact->getMatched() << std::endl;
				thisContact->getForce(1,force);
				thisContact->getNormal(normal);
				thisContact->getPrimaryDirection(primaryDir);
				thisContact->getSecondaryDirection(secondaryDir);
				thisContact->getSlidingVelocity(slidingVel1Dir,slidingVel2Dir);
				VxContactMaterial* thisContactMaterialProps = thisContact->getContactMaterial();
				
				// find the other part
				Vx::VxPart* p1;
				Vx::VxPart* p2;
				thisContact->getPartPair(&p1,&p2);
				int idx2 = -1000;
				int idy2 = -1000;

				for (int p=j+1; p<4; p++)
					if ( p2 == cornerMasses[p]->getVxPart() )
					{
						idx2 = p;
						fprintf(outContact, "%g \t (CM %d) \t (CM %d) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t %g  \t %g  \t %g \t %g \t %g  \t %g  \t %g  \t %g \n", currentTime, j, idx2, force.x(),force.y(),force.z(), normal[0],normal[1],normal[2], primaryDir[0],primaryDir[1],primaryDir[2], secondaryDir[0],secondaryDir[1],secondaryDir[2], thisContact->getPenetration(), slidingVel1Dir, slidingVel2Dir, thisContactMaterialProps->getCompliance() ,thisContactMaterialProps->getDamping(), thisContactMaterialProps->getAdhesiveForce(), thisContactMaterialProps->getRestitution(), thisContactMaterialProps->getRestitutionThreshold() );  
						break;
					} 

				if (idx2==-1000) 
					fprintf(outContact, "%g \t (CM %d) \t (debris) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t (%g, %g, %g) \t %g  \t %g  \t %g \t %g \t %g  \t %g  \t %g  \t %g \n", currentTime, j, force.x(),force.y(),force.z(), normal[0],normal[1],normal[2], primaryDir[0],primaryDir[1],primaryDir[2], secondaryDir[0],secondaryDir[1],secondaryDir[2], thisContact->getPenetration(), slidingVel1Dir, slidingVel2Dir, thisContactMaterialProps->getCompliance() ,thisContactMaterialProps->getDamping(), thisContactMaterialProps->getAdhesiveForce(), thisContactMaterialProps->getRestitution(), thisContactMaterialProps->getRestitutionThreshold() );  
			}
		}
	}

}