// Created by Eleonora Botta, September 6, 2016
// Modified by Eleonora Botta, September 21, 2016: set scale for corner masses, 
//    threads display (the drawLines function works differently in 6.7: wants twice part for displaying two lines)

#include "MyLibraryNet/displayThreadsAndCornerMasses.h"

#include <VxMath/Transformation.h>
#include <VxGraphics/ShapeGenerator.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/CollisionGeometry.h>

#include <iostream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

#define MAXGRIDSIZE 100

void displayThreadsAndCornerMasses(VxGraphics::ShapeGenerator *mShapeGenerator, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], int gNetSize, int CMsPresent)
{
	//display threads
    static Vx::VxColor threadColor = Vx::VxColor(1.0,1.0,0.0,1.0); // (red, green, blue, transparency)
    static Vx::VxColor newThreadColor = Vx::VxColor(1.0,0.0,0.0,1.0); // (red, green, blue, transparency)
    static std::vector<Vx::VxVector3> row;
    static std::vector<Vx::VxVector3> col;
    int i,j;

	int NetSize = gNetSize;

    row.resize(2*gNetSize-2);
    col.resize(2*gNetSize-2);

    for (i=0; i<gNetSize; i++)
    {
		int index = 0;
        for (j=0; j<gNetSize; j++)
        {
            row[index].set(partMatrix[i][j]->getVxPart()->getPosition());
			index = index+1;
			if ( (j>0) && (j<gNetSize-1) )
			{
				row[index].set(partMatrix[i][j]->getVxPart()->getPosition());
				index = index+1;
			}
			//mShapeGenerator->drawLine(partMatrix[i][j]->getVxPart()->getPosition(), partMatrix[i][j+1]->getVxPart()->getPosition(), 1.0 , VxGraphics::ShapeGenerator::DEFAULT_PASS, newThreadColor); // this would work, but is slower
        }
        //mShapeGenerator->drawLines(row, 1.0 , VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
    }

    for (j=0; j<gNetSize; j++)
    {
		int index = 0;
        for (i=0; i<gNetSize; i++)
        {
			col[index].set(partMatrix[i][j]->getVxPart()->getPosition());
			index = index+1;
			if ( (i>0) && (i<gNetSize-1) )
			{
				col[index].set(partMatrix[i][j]->getVxPart()->getPosition());
				index = index+1;
			}
		}
        //mShapeGenerator->drawLines(col, 1.0 , VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
    }

	mShapeGenerator->drawLine(cornerMasses[0]->getVxPart()->getPosition(), partMatrix[0][0]->getVxPart()->getPosition(), 1.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
	mShapeGenerator->drawLine(cornerMasses[1]->getVxPart()->getPosition(), partMatrix[NetSize-1][0]->getVxPart()->getPosition(), 1.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
	mShapeGenerator->drawLine(cornerMasses[2]->getVxPart()->getPosition(), partMatrix[0][NetSize-1]->getVxPart()->getPosition(), 1.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
	mShapeGenerator->drawLine(cornerMasses[3]->getVxPart()->getPosition(), partMatrix[NetSize-1][NetSize-1]->getVxPart()->getPosition(), 1.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);

	//display corner masses, if present
	if (CMsPresent==1)
	{
		VxMath::Matrix44 pointTransform; 
		VxReal CMradius;
		for (i=0; i<4; i++)
		{
			CMradius = cornerMasses[i]->getVxPart()->getBoundingSphere().radius();
			pointTransform = VxMath::Transformation::createTranslation(cornerMasses[i]->getVxPart()->getPosition());
			VxMath::Transformation::setScale(pointTransform, Vx::VxVector3(2*CMradius,2*CMradius,2*CMradius));
			mShapeGenerator->drawSphere(pointTransform, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor );
		}
	}
}