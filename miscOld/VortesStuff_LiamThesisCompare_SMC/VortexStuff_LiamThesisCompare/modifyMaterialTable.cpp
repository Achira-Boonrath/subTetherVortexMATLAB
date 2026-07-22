// Modify the contact properties:	define materials to be used for the parts in the simulation,
//									define contact properties when two objects collide. 

// Created by Eleonora Botta, October 14, 2016
// Modified by Eleonora Botta, October 31, 2016: contact materials computed automatically
// Modified by Eleonora Botta, March 14, 2017: added material for cables in modifyMaterialTable, and modified contact material properties with CableMaterial to avoid collision response
// Modified by Eleonora Botta, March 16, 2017: added modifyMaterialTableToAllowForCableCollisions(), that modifies contact material properties with CableMaterial to have collision response after this is called


#include "MyLibraryNet/modifyMaterialTable.h"

#include "MyLibraryNet/readParameter.h"
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxSolverParameters.h>
#include <Vx/VxMaterialTable.h>
#include <Vx/VxContactMaterial.h>
#include <Vx/VxCore.h>
#include <VxSim/VxExtensionFactory.h>

#include <VxSim/VxMaterialTableExtensionICD.h>
#include <VxSim/VxExtensionFactory.h>


#include <Vx/VxRigidBodyResponseModel.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace Vx;
using namespace VxSim;
using namespace VxSim;

/* Eleonora's
Vx::VxMaterialTable* modifyMaterialTable(std::string pathData)
{
	// read constants
	std::stringstream constantsFileName;
	constantsFileName << pathData << "constants.txt" ;
	double PI = readParameter<double>("PI", constantsFileName);

	// read data for simulation (netModel)
	std::stringstream simulationDataFileName;
	simulationDataFileName << pathData << "simulationData.txt";
	int CMsPresent = readParameter<int>("CMsPresent", simulationDataFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	double netYoungModulus = readParameter<double>("netYoungModulus", netDataFileName);
	double netPoissonRatio = readParameter<double>("netPoissonRatio", netDataFileName);
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	double NetSideLength = readParameter<double>("NetSideLength", netDataFileName);
	double netRadius = readParameter<double>("netRadius", netDataFileName);
	double netDensity = readParameter<double>("netDensity", netDataFileName);
	double MeshLength = NetSideLength / (NetSize-1);
	double dampingCoeff = readParameter<double>("csiAxial", netDataFileName);
	
	// read data for corner masses (if CMs present?)
	std::stringstream CMsDataFileName;
	CMsDataFileName << pathData << "CMsData.txt";
	double BulletMass = readParameter<double>("BulletMass", CMsDataFileName);
	double alDensity = readParameter<double>("alDensity", CMsDataFileName);
	double alYoungModulus = readParameter<double>("alYoungModulus", CMsDataFileName);
	double alPoissonRatio = readParameter<double>("alPoissonRatio", CMsDataFileName);
	double cornerThreadLength = readParameter<double>("cornerThreadLength", CMsDataFileName);
	double cornerThreadRadius = readParameter<double>("cornerThreadRadius", CMsDataFileName);
	if ( cornerThreadLength==-1)
		cornerThreadLength = MeshLength*sqrt(2.0);
	std::cout << cornerThreadLength << std::endl;
	double massHalfCornerThread = netDensity * cornerThreadLength * PI * pow(cornerThreadRadius,2) / 2;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// compute radii of net mass (0,0), (0,1), (1,1)
	double massHalfThread = netDensity * MeshLength * PI * pow(netRadius,2) / 2;
	double part11mass = 4 * massHalfThread;
	double part01mass = 3 * massHalfThread;
	double part00mass = 2 * massHalfThread;
	if (CMsPresent==1)
		part00mass = part00mass + massHalfCornerThread;

	double part11radius = pow((0.75 / PI * part11mass / netDensity),(1/3.));
	double part01radius = pow((0.75 / PI * part01mass / netDensity),(1/3.));
	double part00radius = pow((0.75 / PI * part00mass / netDensity),(1/3.));
 
	// compute radius of corner masses
	double CMmass = BulletMass + massHalfCornerThread;
	double CMradius = pow(0.75 * CMmass / PI / alDensity , 1/3.); // x 10 for visualization only; 

	// compute h coefficients depending on material of a certain body
	double netMaterialCoeff = ( 1 - pow(netPoissonRatio,2.0) ) / PI / netYoungModulus;
	double alMaterialCoeff = ( 1 - pow(alPoissonRatio,2.0) ) / PI / alYoungModulus;

	// Compute contact stiffness according to Hertzian theory. 
	double stiffnessCmDebris =  4.0/3.0/PI / (alMaterialCoeff + alMaterialCoeff) * sqrt(CMradius);
	double stiffnessCmCm =  4.0/3.0/PI / (alMaterialCoeff + alMaterialCoeff) * sqrt( CMradius*CMradius/(CMradius+CMradius) );
	double stiffnessPart00Debris =  4.0/3.0/PI / (netMaterialCoeff + alMaterialCoeff) * sqrt(part00radius);
	double stiffnessPart01Debris =  4.0/3.0/PI / (netMaterialCoeff + alMaterialCoeff) * sqrt(part01radius);
	double stiffnessPart11Debris =  4.0/3.0/PI / (netMaterialCoeff + alMaterialCoeff) * sqrt(part11radius);
	double stiffnessPart00Cm =  4.0/3.0/PI / (netMaterialCoeff + alMaterialCoeff) * sqrt( part00radius*CMradius/(part00radius+CMradius) );
	double stiffnessPart01Cm =  4.0/3.0/PI / (netMaterialCoeff + alMaterialCoeff) * sqrt( part01radius*CMradius/(part01radius+CMradius) );
	double stiffnessPart11Cm =  4.0/3.0/PI / (netMaterialCoeff + alMaterialCoeff) * sqrt( part11radius*CMradius/(part11radius+CMradius) );
	double stiffnessPart00Part00 =  4.0/3.0/PI / (netMaterialCoeff + netMaterialCoeff) * sqrt( part00radius*part00radius/(part00radius+part00radius) );
	double stiffnessPart01Part01 =  4.0/3.0/PI / (netMaterialCoeff + netMaterialCoeff) * sqrt( part01radius*part01radius/(part01radius+part01radius) );
	double stiffnessPart11Part11 =  4.0/3.0/PI / (netMaterialCoeff + netMaterialCoeff) * sqrt( part11radius*part11radius/(part11radius+part11radius) );
	double stiffnessPart00Part01 =  4.0/3.0/PI / (netMaterialCoeff + netMaterialCoeff) * sqrt( part00radius*part01radius/(part00radius+part01radius) );
	double stiffnessPart00Part11 =  4.0/3.0/PI / (netMaterialCoeff + netMaterialCoeff) * sqrt( part00radius*part11radius/(part00radius+part11radius) );
	double stiffnessPart01Part11 =  4.0/3.0/PI / (netMaterialCoeff + netMaterialCoeff) * sqrt( part01radius*part11radius/(part01radius+part11radius) );

	// Compute linear stiffness to be used: estimated as nonlinear stiffness*(2*max penetration/3)^(0.5). 
	// Also divided by a factor of 10 to account for the fact that the mass is lumped in the nodes. 
	// Max penetration is assumed = 0.0001 m 
	double deltaMax = 0.0001;
	stiffnessCmDebris = stiffnessCmDebris * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessCmCm = stiffnessCmCm * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart00Debris = stiffnessPart00Debris * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart01Debris = stiffnessPart01Debris * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart11Debris = stiffnessPart11Debris * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart00Cm = stiffnessPart00Cm * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart01Cm = stiffnessPart01Cm * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart11Cm = stiffnessPart11Cm * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart00Part00 = stiffnessPart00Part00 * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart01Part01 = stiffnessPart01Part01 * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart11Part11 = stiffnessPart11Part11 * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart00Part01 = stiffnessPart00Part01 * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart00Part11 = stiffnessPart00Part11 * sqrt(2.0/3.0*deltaMax) / 10 ;
	stiffnessPart01Part11 = stiffnessPart01Part11 * sqrt(2.0/3.0*deltaMax) / 10 ;

	// compute damping coefficients to be used in the linear model 
	double dampingCmDebris = 2 * dampingCoeff * sqrt( stiffnessCmDebris * CMmass );
	double dampingCmCm =   2 * dampingCoeff * sqrt( stiffnessCmCm * CMmass*CMmass/(CMmass+CMmass) );
	double dampingPart00Debris = 2 * dampingCoeff * sqrt( stiffnessPart00Debris * part00mass );
	double dampingPart01Debris = 2 * dampingCoeff * sqrt( stiffnessPart01Debris * part01mass );
	double dampingPart11Debris = 2 * dampingCoeff * sqrt( stiffnessPart11Debris * part11mass );
	double dampingPart00Cm = 2 * dampingCoeff * sqrt (stiffnessPart00Cm * part00mass*CMmass/(part00mass+CMmass) );
	double dampingPart01Cm = 2 * dampingCoeff * sqrt (stiffnessPart01Cm * part01mass*CMmass/(part01mass+CMmass) );
	double dampingPart11Cm = 2 * dampingCoeff * sqrt (stiffnessPart11Cm * part11mass*CMmass/(part11mass+CMmass) );
	double dampingPart00Part00 = 2 * dampingCoeff * sqrt (stiffnessPart00Part00 * part00mass*part00mass/(part00mass+part00mass) );
	double dampingPart01Part01 = 2 * dampingCoeff * sqrt (stiffnessPart01Part01 * part01mass*part01mass/(part01mass+part01mass) );
	double dampingPart11Part11 = 2 * dampingCoeff * sqrt (stiffnessPart11Part11 * part11mass*part11mass/(part11mass+part11mass) );
	double dampingPart00Part01 = 2 * dampingCoeff * sqrt (stiffnessPart00Part01 * part00mass*part01mass/(part00mass+part01mass) );
	double dampingPart00Part11 = 2 * dampingCoeff * sqrt (stiffnessPart00Part11 * part00mass*part11mass/(part00mass+part11mass) );
	double dampingPart01Part11 = 2 * dampingCoeff * sqrt (stiffnessPart01Part11 * part01mass*part11mass/(part01mass+part11mass) );

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// specify a few parameters for friction
	double staticFrictionCoeffAlAl = 0.42;		// from Davis97.
	double staticFrictionCoeffAlNet = 0.19;		// from Davis97
	double staticFrictionCoeffNetNet = 0.20;	// from Davis97.

	double staticFrictionCoeffScale = 1.25;
	double kineticFrictionCoeffAlAl = staticFrictionCoeffAlAl / staticFrictionCoeffScale;
	double kineticFrictionCoeffAlNet = staticFrictionCoeffAlNet / staticFrictionCoeffScale;
	double kineticFrictionCoeffNetNet = staticFrictionCoeffNetNet / staticFrictionCoeffScale;
	
	// get current material table
	Vx::VxMaterialTable* myTable = VxFrame::instance()->getUniverse(0)->getRigidBodyResponseModel()->getMaterialTable();
	
	// Create materials: parameters are changed in ContactMaterials later. ID=0 is default material
	Vx::VxMaterial* SpacecraftMaterial = myTable->registerMaterial("MySpacecraftMaterial");	// (ID=1)
	Vx::VxMaterial* CmMaterial = myTable->registerMaterial("MyCmMaterial");					// (ID=2)
	Vx::VxMaterial* NetMaterial00 = myTable->registerMaterial("MyNetMaterial00");			// (ID=3)
	Vx::VxMaterial* NetMaterial01 = myTable->registerMaterial("MyNetMaterial01");			// (ID=4)
	Vx::VxMaterial* NetMaterial11 = myTable->registerMaterial("MyNetMaterial11");			// (ID=5)
	Vx::VxMaterial* CableMaterial = myTable->registerMaterial("MyCableMaterial");		// (ID=6)

	// Modify properties of all possible contact materials
	VxContactMaterial *cm = myTable->getContactMaterial(SpacecraftMaterial, CmMaterial);
	cm->setCompliance(1.0/stiffnessCmDebris);
	cm->setDamping(dampingCmDebris);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlAl); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial00);
	cm->setCompliance(1.0/stiffnessPart00Debris);
	cm->setDamping(dampingPart00Debris);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial01);
	cm->setCompliance(1.0/stiffnessPart01Debris);
	cm->setDamping(dampingPart01Debris);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial11);
	cm->setCompliance(1.0/stiffnessPart11Debris);
	cm->setDamping(dampingPart11Debris);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(CmMaterial, CmMaterial);
	cm->setCompliance(1.0/stiffnessCmCm);
	cm->setDamping(dampingCmCm);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlAl); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(CmMaterial, NetMaterial00);
	cm->setCompliance(1.0/stiffnessPart00Cm);
	cm->setDamping(dampingPart00Cm);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(CmMaterial, NetMaterial01);
	cm->setCompliance(1.0/stiffnessPart01Cm);
	cm->setDamping(dampingPart01Cm);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(CmMaterial, NetMaterial11);
	cm->setCompliance(1.0/stiffnessPart11Cm);
	cm->setDamping(dampingPart11Cm);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(NetMaterial00, NetMaterial00); 
	cm->setCompliance(1.0/stiffnessPart00Part00);
	cm->setDamping(dampingPart00Part00);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(NetMaterial00, NetMaterial01); 
	cm->setCompliance(1.0/stiffnessPart00Part01);
	cm->setDamping(dampingPart00Part01);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(NetMaterial00, NetMaterial11); 
	cm->setCompliance(1.0/stiffnessPart00Part11);
	cm->setDamping(dampingPart00Part11);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(NetMaterial01, NetMaterial01); 
	cm->setCompliance(1.0/stiffnessPart01Part01);
	cm->setDamping(dampingPart01Part01);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(NetMaterial01, NetMaterial11); 
	cm->setCompliance(1.0/stiffnessPart01Part11);
	cm->setDamping(dampingPart01Part11);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(NetMaterial11, NetMaterial11); 
	cm->setCompliance(1.0/stiffnessPart11Part11);
	cm->setDamping(dampingPart11Part11);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet); 
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale); 
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);		

	cm = myTable->getContactMaterial(SpacecraftMaterial, CableMaterial);
	cm->setCompliance(1.0/0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(CableMaterial, CableMaterial);
	cm->setCompliance(1.0/0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(NetMaterial11, CableMaterial);
	cm->setCompliance(1.0/0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(NetMaterial01, CableMaterial);
	cm->setCompliance(1.0/0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(NetMaterial00, CableMaterial);
	cm->setCompliance(1.0/0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(CmMaterial, CableMaterial);
	cm->setCompliance(1.0/0.0000001);
	cm->setDamping(0.0000001);
	
	const char* MaterialTableForFile = "MaterialTable" ; 
	myTable->dump(MaterialTableForFile); // print material table to file
	//myTable->clear(true); clears the material table
	//myTable->dump(NULL); prints the material table to the console

	return myTable;
}
*/


Vx::VxMaterialTable* modifyMaterialTable(std::string pathData)
{
	// read constants
	std::stringstream constantsFileName;
	constantsFileName << pathData << "constants.txt";
	double PI = readParameter<double>("PI", constantsFileName);

	// read data for simulation (netModel)
	std::stringstream simulationDataFileName;
	simulationDataFileName << pathData << "simulationData.txt";
	int CMsPresent = readParameter<int>("CMsPresent", simulationDataFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	double netYoungModulus = readParameter<double>("netYoungModulus", netDataFileName);
	double netPoissonRatio = readParameter<double>("netPoissonRatio", netDataFileName);
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	double NetSideLength = readParameter<double>("NetSideLength", netDataFileName);
	double netRadius = readParameter<double>("netRadius", netDataFileName);
	double netDensity = readParameter<double>("netDensity", netDataFileName);
	double MeshLength = NetSideLength / (NetSize - 1);
	double dampingCoeff = readParameter<double>("csiAxial", netDataFileName);

	// read data for corner masses (if CMs present?)
	std::stringstream CMsDataFileName;
	CMsDataFileName << pathData << "CMsData.txt";
	double BulletMass = readParameter<double>("BulletMass", CMsDataFileName);
	double alDensity = readParameter<double>("alDensity", CMsDataFileName);
	double alYoungModulus = readParameter<double>("alYoungModulus", CMsDataFileName);
	double alPoissonRatio = readParameter<double>("alPoissonRatio", CMsDataFileName);
	double cornerThreadLength = readParameter<double>("cornerThreadLength", CMsDataFileName);
	double cornerThreadRadius = readParameter<double>("cornerThreadRadius", CMsDataFileName);
	if (cornerThreadLength == -1)
		cornerThreadLength = MeshLength * sqrt(2.0);
	std::cout << cornerThreadLength << std::endl;
	double massHalfCornerThread = netDensity * cornerThreadLength * PI * pow(cornerThreadRadius, 2) / 2;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// compute radii of net mass (0,0), (0,1), (1,1)
	double massHalfThread = netDensity * MeshLength * PI * pow(netRadius, 2) / 2;
	double part11mass = 4 * massHalfThread;
	double part01mass = 3 * massHalfThread;
	double part00mass = 2 * massHalfThread;
	if (CMsPresent == 1)
		part00mass = part00mass + massHalfCornerThread;

	double part11radius = pow((0.75 / PI * part11mass / netDensity), (1 / 3.));
	double part01radius = pow((0.75 / PI * part01mass / netDensity), (1 / 3.));
	double part00radius = pow((0.75 / PI * part00mass / netDensity), (1 / 3.));

	// compute radius of corner masses
	double CMmass = BulletMass + massHalfCornerThread;
	double CMradius = pow(0.75 * CMmass / PI / alDensity, 1 / 3.); // x 10 for visualization only; 

																   // compute h coefficients depending on material of a certain body
	double netMaterialCoeff = (1 - pow(netPoissonRatio, 2.0)) / PI / netYoungModulus;
	double alMaterialCoeff = (1 - pow(alPoissonRatio, 2.0)) / PI / alYoungModulus;

	// Compute contact stiffness according to Hertzian theory. 
	double stiffnessCmDebris = 4.0 / 3.0 / PI / (alMaterialCoeff + alMaterialCoeff) * sqrt(CMradius);
	double stiffnessCmCm = 4.0 / 3.0 / PI / (alMaterialCoeff + alMaterialCoeff) * sqrt(CMradius*CMradius / (CMradius + CMradius));
	double stiffnessPart00Debris = 4.0 / 3.0 / PI / (netMaterialCoeff + alMaterialCoeff) * sqrt(part00radius);
	double stiffnessPart01Debris = 4.0 / 3.0 / PI / (netMaterialCoeff + alMaterialCoeff) * sqrt(part01radius);
	double stiffnessPart11Debris = 4.0 / 3.0 / PI / (netMaterialCoeff + alMaterialCoeff) * sqrt(part11radius);
	double stiffnessPart00Cm = 4.0 / 3.0 / PI / (netMaterialCoeff + alMaterialCoeff) * sqrt(part00radius*CMradius / (part00radius + CMradius));
	double stiffnessPart01Cm = 4.0 / 3.0 / PI / (netMaterialCoeff + alMaterialCoeff) * sqrt(part01radius*CMradius / (part01radius + CMradius));
	double stiffnessPart11Cm = 4.0 / 3.0 / PI / (netMaterialCoeff + alMaterialCoeff) * sqrt(part11radius*CMradius / (part11radius + CMradius));
	double stiffnessPart00Part00 = 4.0 / 3.0 / PI / (netMaterialCoeff + netMaterialCoeff) * sqrt(part00radius*part00radius / (part00radius + part00radius));
	double stiffnessPart01Part01 = 4.0 / 3.0 / PI / (netMaterialCoeff + netMaterialCoeff) * sqrt(part01radius*part01radius / (part01radius + part01radius));
	double stiffnessPart11Part11 = 4.0 / 3.0 / PI / (netMaterialCoeff + netMaterialCoeff) * sqrt(part11radius*part11radius / (part11radius + part11radius));
	double stiffnessPart00Part01 = 4.0 / 3.0 / PI / (netMaterialCoeff + netMaterialCoeff) * sqrt(part00radius*part01radius / (part00radius + part01radius));
	double stiffnessPart00Part11 = 4.0 / 3.0 / PI / (netMaterialCoeff + netMaterialCoeff) * sqrt(part00radius*part11radius / (part00radius + part11radius));
	double stiffnessPart01Part11 = 4.0 / 3.0 / PI / (netMaterialCoeff + netMaterialCoeff) * sqrt(part01radius*part11radius / (part01radius + part11radius));

	// Compute linear stiffness to be used: estimated as nonlinear stiffness*(2*max penetration/3)^(0.5). 
	// Also divided by a factor of 10 to account for the fact that the mass is lumped in the nodes. 
	// Max penetration is assumed = 0.0001 m 
	double deltaMax = 0.0001;
	stiffnessCmDebris = stiffnessCmDebris * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessCmCm = stiffnessCmCm * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart00Debris = stiffnessPart00Debris * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart01Debris = stiffnessPart01Debris * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart11Debris = stiffnessPart11Debris * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart00Cm = stiffnessPart00Cm * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart01Cm = stiffnessPart01Cm * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart11Cm = stiffnessPart11Cm * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart00Part00 = stiffnessPart00Part00 * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart01Part01 = stiffnessPart01Part01 * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart11Part11 = stiffnessPart11Part11 * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart00Part01 = stiffnessPart00Part01 * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart00Part11 = stiffnessPart00Part11 * sqrt(2.0 / 3.0*deltaMax) / 10;
	stiffnessPart01Part11 = stiffnessPart01Part11 * sqrt(2.0 / 3.0*deltaMax) / 10;

	// compute damping coefficients to be used in the linear model 
	double dampingCmDebris = 2 * dampingCoeff * sqrt(stiffnessCmDebris * CMmass);
	double dampingCmCm = 2 * dampingCoeff * sqrt(stiffnessCmCm * CMmass*CMmass / (CMmass + CMmass));
	double dampingPart00Debris = 2 * dampingCoeff * sqrt(stiffnessPart00Debris * part00mass);
	double dampingPart01Debris = 2 * dampingCoeff * sqrt(stiffnessPart01Debris * part01mass);
	double dampingPart11Debris = 2 * dampingCoeff * sqrt(stiffnessPart11Debris * part11mass);
	double dampingPart00Cm = 2 * dampingCoeff * sqrt(stiffnessPart00Cm * part00mass*CMmass / (part00mass + CMmass));
	double dampingPart01Cm = 2 * dampingCoeff * sqrt(stiffnessPart01Cm * part01mass*CMmass / (part01mass + CMmass));
	double dampingPart11Cm = 2 * dampingCoeff * sqrt(stiffnessPart11Cm * part11mass*CMmass / (part11mass + CMmass));
	double dampingPart00Part00 = 2 * dampingCoeff * sqrt(stiffnessPart00Part00 * part00mass*part00mass / (part00mass + part00mass));
	double dampingPart01Part01 = 2 * dampingCoeff * sqrt(stiffnessPart01Part01 * part01mass*part01mass / (part01mass + part01mass));
	double dampingPart11Part11 = 2 * dampingCoeff * sqrt(stiffnessPart11Part11 * part11mass*part11mass / (part11mass + part11mass));
	double dampingPart00Part01 = 2 * dampingCoeff * sqrt(stiffnessPart00Part01 * part00mass*part01mass / (part00mass + part01mass));
	double dampingPart00Part11 = 2 * dampingCoeff * sqrt(stiffnessPart00Part11 * part00mass*part11mass / (part00mass + part11mass));
	double dampingPart01Part11 = 2 * dampingCoeff * sqrt(stiffnessPart01Part11 * part01mass*part11mass / (part01mass + part11mass));

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// specify a few parameters for friction
	double staticFrictionCoeffAlAl = 0.42;		// from Davis97.
	double staticFrictionCoeffAlNet = 0.19;		// from Davis97
	double staticFrictionCoeffNetNet = 0.20;	// from Davis97.

	double staticFrictionCoeffScale = 1.25;
	double kineticFrictionCoeffAlAl = staticFrictionCoeffAlAl / staticFrictionCoeffScale;
	double kineticFrictionCoeffAlNet = staticFrictionCoeffAlNet / staticFrictionCoeffScale;
	double kineticFrictionCoeffNetNet = staticFrictionCoeffNetNet / staticFrictionCoeffScale;

	// get current material table
	Vx::VxMaterialTable* myTable = VxFrame::instance()->getUniverse(0)->getRigidBodyResponseModel()->getMaterialTable();
	//VxSim::VxExtension* materialTable = VxSim::VxExtensionFactory::create(VxSim::VxMaterialTableExtensionICD::kPluginCategoryName, VxSim::VxMaterialTableExtensionICD::kExtensionFeatureName);

	// Create materials: parameters are changed in ContactMaterials later. ID=0 is default material
	Vx::VxMaterial* SpacecraftMaterial = myTable->registerMaterial("MySpacecraftMaterial");	// (ID=1)
	Vx::VxMaterial* CmMaterial = myTable->registerMaterial("MyCmMaterial");					// (ID=2)
	Vx::VxMaterial* NetMaterial00 = myTable->registerMaterial("MyNetMaterial00");			// (ID=3)
	Vx::VxMaterial* NetMaterial01 = myTable->registerMaterial("MyNetMaterial01");			// (ID=4)
	Vx::VxMaterial* NetMaterial11 = myTable->registerMaterial("MyNetMaterial11");			// (ID=5)
	Vx::VxMaterial* CableMaterial = myTable->registerMaterial("MyCableMaterial");		// (ID=6)

																						// Modify properties of all possible contact materials
	VxContactMaterial *cm = myTable->getContactMaterial(SpacecraftMaterial, CmMaterial);
	cm->setCompliance(1.0 / stiffnessCmDebris);
	cm->setDamping(dampingCmDebris);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlAl);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial00);
	cm->setCompliance(1.0 / stiffnessPart00Debris);
	cm->setDamping(dampingPart00Debris);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial01);
	cm->setCompliance(1.0 / stiffnessPart01Debris);
	cm->setDamping(dampingPart01Debris);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial11);
	cm->setCompliance(1.0 / stiffnessPart11Debris);
	cm->setDamping(dampingPart11Debris);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(CmMaterial, CmMaterial);
	cm->setCompliance(1.0 / stiffnessCmCm);
	cm->setDamping(dampingCmCm);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlAl);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(CmMaterial, NetMaterial00);
	cm->setCompliance(1.0 / stiffnessPart00Cm);
	cm->setDamping(dampingPart00Cm);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(CmMaterial, NetMaterial01);
	cm->setCompliance(1.0 / stiffnessPart01Cm);
	cm->setDamping(dampingPart01Cm);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(CmMaterial, NetMaterial11);
	cm->setCompliance(1.0 / stiffnessPart11Cm);
	cm->setDamping(dampingPart11Cm);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffAlNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(NetMaterial00, NetMaterial00);
	cm->setCompliance(1.0 / stiffnessPart00Part00);
	cm->setDamping(dampingPart00Part00);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(NetMaterial00, NetMaterial01);
	cm->setCompliance(1.0 / stiffnessPart00Part01);
	cm->setDamping(dampingPart00Part01);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(NetMaterial00, NetMaterial11);
	cm->setCompliance(1.0 / stiffnessPart00Part11);
	cm->setDamping(dampingPart00Part11);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(NetMaterial01, NetMaterial01);
	cm->setCompliance(1.0 / stiffnessPart01Part01);
	cm->setDamping(dampingPart01Part01);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(NetMaterial01, NetMaterial11);
	cm->setCompliance(1.0 / stiffnessPart01Part11);
	cm->setDamping(dampingPart01Part11);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(NetMaterial11, NetMaterial11);
	cm->setCompliance(1.0 / stiffnessPart11Part11);
	cm->setDamping(dampingPart11Part11);
	cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, kineticFrictionCoeffNetNet);
	cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear, staticFrictionCoeffScale);
	cm->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
	cm->setSlip(VxMaterialBase::kFrictionAxisLinear, 0.000001);

	cm = myTable->getContactMaterial(SpacecraftMaterial, CableMaterial);
	cm->setCompliance(1.0 / 0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(CableMaterial, CableMaterial);
	cm->setCompliance(1.0 / 0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(NetMaterial11, CableMaterial);
	cm->setCompliance(1.0 / 0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(NetMaterial01, CableMaterial);
	cm->setCompliance(1.0 / 0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(NetMaterial00, CableMaterial);
	cm->setCompliance(1.0 / 0.0000001);
	cm->setDamping(0.0000001);

	cm = myTable->getContactMaterial(CmMaterial, CableMaterial);
	cm->setCompliance(1.0 / 0.0000001);
	cm->setDamping(0.0000001);

	const char* MaterialTableForFile = "MaterialTable";

	return myTable;
}

void modifyMaterialTableToAllowForCableCollisions()
{
	std::cout << "I am changing the CableMaterial contact properties" << std::endl; 
	// Get materialTable
	Vx::VxMaterialTable* materialTable =  VxFrame::instance()->getUniverse(0)->getRigidBodyResponseModel()->getMaterialTable();
	// Get materials
	VxMaterial* SpacecraftMaterial = materialTable->getMaterial(1);
	VxMaterial* CmMaterial = materialTable->getMaterial(2);
	VxMaterial* NetMaterial00 = materialTable->getMaterial(3);
	VxMaterial* NetMaterial01 = materialTable->getMaterial(4);
	VxMaterial* NetMaterial11 = materialTable->getMaterial(5);
	VxMaterial* CableMaterial = materialTable->getMaterial(6);
	// Change all contact materials with cable to default to have collision response of cables		
	VxContactMaterial *cm = materialTable->getContactMaterial(SpacecraftMaterial, CableMaterial);
	cm->setCompliance(0);
	cm->setDamping(0);

	cm = materialTable->getContactMaterial(CableMaterial, CableMaterial);
	cm->setCompliance(0);
	cm->setDamping(0);

	cm = materialTable->getContactMaterial(NetMaterial11, CableMaterial);
	cm->setCompliance(0);
	cm->setDamping(0);

	cm = materialTable->getContactMaterial(NetMaterial01, CableMaterial);
	cm->setCompliance(0);
	cm->setDamping(0);

	cm = materialTable->getContactMaterial(NetMaterial00, CableMaterial);
	cm->setCompliance(0);
	cm->setDamping(0);

	cm = materialTable->getContactMaterial(CmMaterial, CableMaterial);
	cm->setCompliance(0);
	cm->setDamping(0);

	return;
}



/* // This was for before, when I had some fixed values for the contact properties. 
Vx::VxMaterialTable* modifyMaterialTable()
{
	Vx::VxMaterialTable* myTable = VxFrame::instance()->getUniverse(0)->getMaterialTable(); // gets material table 	

	// Create Spacecraft Material (ID=1): parameters are changed in ContactMaterials later
	Vx::VxMaterial* SpacecraftMaterial = myTable->registerMaterial("MySpacecraftMaterial");
	// normal direction
	SpacecraftMaterial->setCompliance(1.0e-6f); 
    SpacecraftMaterial->setDamping(1.0e005f);
	// tangential direction
	SpacecraftMaterial->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    SpacecraftMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.45*4.0/5.0); //Al-Al
	SpacecraftMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25
	SpacecraftMaterial->setSlip(VxMaterialBase::kFrictionAxisLinearPrimary, 0.000001);		
		
	// Create Corner Masses Material (ID=2):  parameters are changed in ContactMaterials later
	Vx::VxMaterial* CmMaterial = myTable->registerMaterial("MyCmMaterial");
	// normal direction
	CmMaterial->setCompliance(1.0e-6f); 
    CmMaterial->setDamping(1.0e005f);	
	// tangential direction
	CmMaterial->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    CmMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.45*4.0/5.0); //Al-Al
	CmMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25
	CmMaterial->setSlip(VxMaterialBase::kFrictionAxisLinearPrimary, 0.000001);	

	// Create Net Material 00 (ID=3):  parameters are changed in ContactMaterials later
    Vx::VxMaterial *NetMaterial00 = myTable->registerMaterial("MyNetMaterial00");
	// normal direction
	NetMaterial00->setCompliance(1.0e-6f); 
    NetMaterial00->setDamping(1.0e005f);	
	// tangential direction
	NetMaterial00->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    NetMaterial00->setSlip(VxMaterialBase::kFrictionAxisLinearPrimary, 0.000001);	
	NetMaterial00->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); //Nylon-Nylon
	NetMaterial00->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25
	
	// Create Net Material 01 (ID=4):  parameters are changed in ContactMaterials later
    Vx::VxMaterial *NetMaterial01 = myTable->registerMaterial("MyNetMaterial01");
	// normal direction
	NetMaterial01->setCompliance(1.0e-6f); 
    NetMaterial01->setDamping(1.0e005f);	
	// tangential direction
	NetMaterial01->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    NetMaterial01->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); //Nylon-Nylon
	NetMaterial01->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25
	NetMaterial01->setSlip(VxMaterialBase::kFrictionAxisLinearPrimary, 0.000001);

	// Create Net Material 11 (ID=5):  parameters are changed in ContactMaterials later
    Vx::VxMaterial *NetMaterial11 = myTable->registerMaterial("MyNetMaterial11");
	// normal direction
	NetMaterial11->setCompliance(1.0e-6f); 
    NetMaterial11->setDamping(1.0e005f);	
	// tangential direction
	NetMaterial11->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    NetMaterial11->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); //Nylon-Nylon
	NetMaterial11->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25
	NetMaterial11->setSlip(VxMaterialBase::kFrictionAxisLinearPrimary, 0.000001);

	//if ( (NetSize == 21) & ( NetSideLength==20.0 ) & (netRadius==0.0005) )
	//{
		// Computed from Hertz theory, according to contact geometry
		VxContactMaterial *cm = myTable->getContactMaterial(SpacecraftMaterial, CmMaterial);
		cm->setCompliance(1.0/9711968257);
		cm->setDamping(1396);
		cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.45*4.0/5.0); //Al-Al
		cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial00);
		cm->setCompliance(1.0/4590678135);
		cm->setDamping(70);
		cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.15*4.0/5.0); // Al-Nylon
		cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial01);
		cm->setCompliance(1.0/4240620319);
		cm->setDamping(53);
		cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.15*4.0/5.0); // Al-Nylon
		cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial11);
		cm->setCompliance(1.0/4448900501);
		cm->setDamping(62);
		cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.15*4.0/5.0); // Al-Nylon
		cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(CmMaterial, CmMaterial);
		cm->setCompliance(1.0/6867398613);
		cm->setDamping(830);
		cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 1.1*4.0/5.0); // Al-Al
		cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(CmMaterial, NetMaterial00);
		cm->setCompliance(1.0/4161454582);
		cm->setDamping(66);
		cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.15*4.0/5.0); // Al-Nylon
		cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(CmMaterial, NetMaterial01);
		cm->setCompliance(1.0/3895394424);
		cm->setDamping(50);
		cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.15*4.0/5.0); // Al-Nylon
		cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(CmMaterial, NetMaterial11);
		cm->setCompliance(1.0/4054971687);
		cm->setDamping(59);
		cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.15*4.0/5.0); // Al-Nylon
		cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(NetMaterial00, NetMaterial00); 
		cm->setCompliance(1.0/3295141625);
		cm->setDamping(42);
		SpacecraftMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); // Nylon-Nylon
		SpacecraftMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(NetMaterial00, NetMaterial01); 
		cm->setCompliance(1.0/3162044633);
		cm->setDamping(36);
		SpacecraftMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); // Nylon-Nylon
		SpacecraftMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(NetMaterial00, NetMaterial11); 
		cm->setCompliance(1.0/3243061445);
		cm->setDamping(39);
		SpacecraftMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); // Nylon-Nylon
		SpacecraftMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(NetMaterial01, NetMaterial01); 
		cm->setCompliance(1.0/3043873719);
		cm->setDamping(32);
		SpacecraftMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); // Nylon-Nylon
		SpacecraftMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(NetMaterial01, NetMaterial11); 
		cm->setCompliance(1.0/3115937745);
		cm->setDamping(34);
		SpacecraftMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); // Nylon-Nylon
		SpacecraftMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

		cm = myTable->getContactMaterial(NetMaterial11, NetMaterial11); 
		cm->setCompliance(1.0/3193375095);
		cm->setDamping(37);
		SpacecraftMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); // Nylon-Nylon
		SpacecraftMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25
	//}
	//else
	//	 Vx::Vx::LogError(0, "Contact materials cannot be computed.");

	const char* MaterialTableForFile="MaterialTable" ; 
	myTable->dump (MaterialTableForFile); // print material table to file

	return myTable;
}
*/