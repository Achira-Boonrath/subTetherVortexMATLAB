// Created by Eleonora Botta, September 10, 2016

#ifndef WRITENATURALFREQUENCIES_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define WRITENATURALFREQUENCIES_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <fstream>
#include <sstream>

// define headers of cpp functions
void writeNaturalFrequencies(std::string pathData, float wn1Axial, float wn1BendingRot, float wn1BendingTransl);

#endif