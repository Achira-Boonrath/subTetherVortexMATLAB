// Created by Eleonora Botta, June 16, 2017

// Change gravity at run-time

#include "MyLibraryNet/changeGravity.h"

#include <string>
#include <stdio.h>
#include <fstream>

void changeGravity() (std::ifstream gravityReadFileName)
{
	//if ( (targetPresent==1) && (targetSpacecraft=="envisat") )
			{
				if (gravityReadFileName.is_open()) 
				{
					std::string line,lineTemp,var3;
					std::getline(gravityReadFileName,line);  // read one line
					std::stringstream  lineStream(line);
					
					std::getline(lineStream,lineTemp,'\t');
					VxReal readTime = stod(lineTemp); 
					std::getline(lineStream,lineTemp,'\t');
					VxReal gravityX = stod(lineTemp);         // put first quantity in a variable
					std::getline(lineStream,lineTemp,'\t');
					VxReal gravityY = stod(lineTemp);
					std::getline(lineStream,lineTemp,'\t');
					VxReal gravityZ = stod(lineTemp);
					std::cout<<readTime<<";"<<gravityX <<";"<<gravityY <<";"<<gravityZ <<"\n";
			
					// VxReal gravityX, gravityY, gravityZ;
					// read  gravityX, gravityY, gravityZ from file, for a certain t. 
					VxFrame::instance()->getUniverse(0)->setGravity( gravityX, gravityY, gravityZ ); 

}