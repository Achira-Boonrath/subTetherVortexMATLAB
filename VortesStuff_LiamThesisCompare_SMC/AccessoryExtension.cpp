// Created by Andreas Enzenhöfer, September 21, 2016: extensions for handling graphics
// Modified by Eleonora Botta, September 23, 2016: chaser graphics
// Modified by Eleonora Botta, September 29, 2016: graphics for winch.
// Modified by Eleonora Botta, March 3, 2017: pass netModel
// Modified by Eleonora Botta, April 17, 2017: commented KeyboardHelper, VxSimulatorModuleFactory for Vortex Studio 2017a
// Modified by Eleonora Botta, May 14, 2020: added visualization of std closing mechanism

#include "MyLibraryNet/AccessoryExtension.h"
#include "MyLibraryNet/DisplayThreadsAndCornerMasses.h"
#include "MyLibraryNet/displayScene.h"
#include <VxGraphics/ShapeGenerator.h>
// EB 24 jan 2020 #include <VxGraphics/SceneGraph.h>
//#include <KeyboardHelper/KeyboardHelpDisplayICD.h>
//#include <KeyboardHelper/KeyboardNotificationDisplayICD.h>

#include <Vx/VxSmartPtr.h>
#include <VxSim/VxSmartInterface.h>
//#include <VxSim/VxSimulatorModuleFactory.h>
#include <VxSim/VxExtensionFactory.h>
#include <VxGraphics/GraphicsModule.h>
#include <VxGraphicsPlugins/GraphicsModuleICD_OSG.h>
#include <VxGraphics/Services.h>

#include <VxPluginSystem/VxPluginManager.h>
#include <VxDynamics/Part.h>

AccessoryExtension::~AccessoryExtension()
{
    // does nothing
}

AccessoryExtension::AccessoryExtension(VxSim::VxExtension * proxy)
    : VxSim::IExtension(proxy)
    , VxGraphics::IGraphic(proxy)
    , VxGraphics::IAccessory(proxy)
{  }

/* // Andreas'
void AccessoryExtension::setConfiguration (VxSim::VxSmartInterface<VxDynamics::Part>** partMatrix, 
    VxSim::VxSmartInterface<VxDynamics::Part>* cornerMasses, int* gNetSize, int* CMsPresent)
{
    this->partMatrix = partMatrix;
    this->cornerMasses = cornerMasses;
    this->gNetSize = gNetSize;
    this->CMsPresent = CMsPresent;
}
*/


void AccessoryExtension::setConfiguration (VxSim::VxSmartInterface<VxDynamics::Part>* partVector,
	 VxSim::VxSmartInterface<VxDynamics::Part>* chaserPartPtr,float* chaserSideLength, VxSim::VxSmartInterface<VxDynamics::Part>* tetheredTargetPartPtr, int* numNode, int* TetherType)
{
    this->partVector = partVector;
	this->chaserPartPtr = chaserPartPtr;
	this->chaserSideLength = chaserSideLength;
	this->tetheredTargetPartPtr = tetheredTargetPartPtr;

	//AB
	this->numNode = numNode;
	this->TetherType = TetherType;
   
	
}

// Called after modules Update().
// Called automatically at each t step. Only if extension added to main
void AccessoryExtension::drawAccessory(VxGraphics::ShapeGenerator& shapeGenerator)
{
    //displayThreadsAndCornerMasses(&shapeGenerator, partMatrix, cornerMasses, *gNetSize, *CMsPresent); // Andreas'
	//displayScene(&shapeGenerator, partMatrix, cornerMasses, *gNetSize, *CMsPresent, *chaserPresent, chaserPartPtr, *chaserSideLength, winchPartPtr, *winchRadius, *netModel);
	displayScene(&shapeGenerator, partVector, chaserPartPtr, *chaserSideLength, tetheredTargetPartPtr, *numNode, *TetherType);
}