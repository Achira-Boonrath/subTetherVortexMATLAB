// Created by Eleonora Botta, September 19, 2016

#ifndef CHECKDATA_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define CHECKDATA_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/readParameter.h"

#include <Vx/VxMessage.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace Vx;

// define headers of cpp functions
void checkData( std::string pathData);

#endif