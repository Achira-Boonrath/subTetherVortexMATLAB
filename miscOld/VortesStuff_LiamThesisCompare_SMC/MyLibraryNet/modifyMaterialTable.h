// Created by Eleonora Botta, October 14, 2016
// Modified by Eleonora Botta, October 31, 2016: contact materials computed automatically
// Modified by Eleonora Botta, March 16, 2017: added modifyMaterialTableToAllowForCableCollisions(), that modifies contact material properties with CableMaterial to have collision response after this is called

#ifndef MODIFYMATERIALTABLE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define MODIFYMATERIALTABLE_H

// Include all libraries needed by cpp files in this header
#include <Vx/VxMaterialTable.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace Vx;

// define headers of cpp functions

// with materials defined in input files
Vx::VxMaterialTable* modifyMaterialTable(std::string pathData);

// with some default materials
//Vx::VxMaterialTable* modifyMaterialTable();

// to allow for collision response of net cables
void modifyMaterialTableToAllowForCableCollisions();



#endif
