// Created by Eleonora Botta, September 19, 2016
// Modified by Eleonora Botta, September 25, 2016: pass winchPart.
// Modified by Eleonora Botta, January 19, 2017: add possibility of multiple tethers.
// Modified by Eleonora Botta, February 16, 2017: possibility to use different configuration for 3 rings
// Modified by Eleonora Botta, February 16, 2017: possibility to use different configuration for 3 rings
// Modified by Eleonora Botta, February 17, 2017: possibility to have 2 tethers configuration 


#ifndef ADDTETHERTOCT_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ADDTETHERTOCT_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/readArray.h"
//#include "MyLibraryNet/addTarget.h"

#include <VxSim/VxExtensionFactory.h>
#include <VxSim/VxFactoryKey.h>
#include <VxSim/VxSmartInterface.h>
#include <CableSystems/CableSystemsICD.h>
#include <CableSystems/DynamicsICD.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/Part.h>
#include <VxData/Container.h>
#include <VxData/FieldBase.h>
//#include <Vx/VxConnectionFactory.h>

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

#define MAXGRIDSIZE 100

// define headers of cpp functions
void addTetherToCT(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> *partMatrix, VxSmartInterface<Part>* tetheredTargetPart);

void addTethersWith4Rings(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter);

void addTethersWith3Rings(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter);

void addTethersWith3RingsNew(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter);

void addTethersWith3RingsConfiguration2(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter);

void add2Tethers(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter);

#endif