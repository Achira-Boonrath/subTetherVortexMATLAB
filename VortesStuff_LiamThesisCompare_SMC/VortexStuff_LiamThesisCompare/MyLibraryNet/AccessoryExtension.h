// Created by Andreas Enzenhöfer, September 21, 2016: extensions for handling graphics
// Modified by Eleonora Botta, September 23, 2016: chaser graphics
// Modified by Eleonora Botta, September 29, 2016: graphics for winch.
// Modified by Eleonora Botta, March 3, 2017: pass netModel
// Modified by Eleonora Botta, May 14, 2020: added visualization of std closing mechanism

#ifndef _ACCESSORY_EXTENSION_H_
#define _ACCESSORY_EXTENSION_H_

//#include <VxSim/ISimulatorModule.h>
#include <VxSim/IExtension.h>
#include <VxSim/VxSmartInterface.h>
#include <VxGraphics/IAccessory.h>
#include <VxSim/VxExtensionFactory.h>
#include <set>
#include <VxGraphics/IGraphic.h>
#include <VxDynamics/Part.h>

class AccessoryExtension : public VxSim::IExtension, public VxGraphics::IGraphic, public VxGraphics::IAccessory
{
    VxSim::VxSmartInterface<VxDynamics::Part>** partMatrix;
	VxSim::VxSmartInterface<VxDynamics::Part>* partVector;
    VxSim::VxSmartInterface<VxDynamics::Part>* cornerMasses;
    int* gNetSize;
    int* CMsPresent;
	int* chaserPresent;
	VxSim::VxSmartInterface<VxDynamics::Part>* chaserPartPtr;
	float* chaserSideLength;
	VxSim::VxSmartInterface<VxDynamics::Part>* tetheredTargetPartPtr;
	float* winchRadius;
	int* netModel; 
	int* closingMechanismType;
	int* numNode;
	int* TetherType;
	
public:

    ~AccessoryExtension();

    AccessoryExtension(VxSim::VxExtension * proxy);

    //void setConfiguration (VxSim::VxSmartInterface<VxDynamics::Part>** partMatrix, VxSim::VxSmartInterface<VxDynamics::Part>* cornerMasses, int* gNetSize, int* CMsPresent); //Andreas'
	//void setConfiguration(VxSim::VxSmartInterface<VxDynamics::Part>** partMatrix, VxSim::VxSmartInterface<VxDynamics::Part>* cornerMasses, int* gNetSize, int* CMsPresent, int* chaserPresent, VxSim::VxSmartInterface<VxDynamics::Part>* chaserPartPtr, float* chaserSideLength, VxSim::VxSmartInterface<VxDynamics::Part>* winchPartPtr, float* winchRadius, int* netModel);
	void setConfiguration(VxSim::VxSmartInterface<VxDynamics::Part>* partVector,
		VxSim::VxSmartInterface<VxDynamics::Part>* chaserPartPtr, float* chaserSideLength, VxSim::VxSmartInterface<VxDynamics::Part>* tetheredTargetPartPtr, int* numNode, int* TetherType);
    virtual void drawAccessory(VxGraphics::ShapeGenerator& shapeGenerator);
};

#endif //_ACCESSORY_EXTENSION_H_