#include <KeyboardHelper/KeyboardHelpDisplayICD.h>
#include <KeyboardHelper/KeyboardNotificationDisplayICD.h>

#include <Vx/VxSmartPtr.h>
#include <VxData/Vector.h>
#include <VxData/FieldArray.h>

#include <VxSim/VxApplication.h>
#include <VxSim/VxDynamicsModuleICD.h>
#include <VxSim/VxSimulatorModuleFactory.h>
#include <VxSim/VxExtensionFactory.h>
#include <VxSim/FindInterface.h>
#include <VxSim/IKeyboard.h>
#include <VxSim/VxObjectHelpers.h>
#include <VxSim/VxObjectSerializer.h>

#include <VxGraphics/Context.h>
#include <VxGraphics/ICamera.h>
#include <VxGraphics/GraphicsModule.h>
#include <VxGraphics/Viewport.h>
#include <VxGraphicsPlugins/PerspectiveICD.h>
#include <VxGraphicsPlugins/DynamicsVisualizerICD.h>
#include <VxGraphicsPlugins/GraphicsModuleICD_OSG.h>

#include <VxGraphics/SceneGraph.h>
#include <VxGraphics/Services.h>
#include <VxGraphics/ShapeGenerator.h>

#include <VxPluginSystem/VxPluginManager.h>

#include <Vx/VxAssembly.h>
#include <Vx/VxBox.h>
#include <Vx/VxCylinder.h>
#include <Vx/VxSphere.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxCompositeCollisionGeometry.h>
#include <Vx/VxTriangleMeshBVTree.h>
#include <Vx/VxPart.h>
#include <Vx/VxTransform.h>
#include <VxSim/VxApplication.h>
#include <VxSim/VxMechanism.h>

// Include the used constraints
#include <Vx/VxCylindrical.h>
#include <Vx/VxGearRatio.h>
#include <Vx/VxHinge.h>
#include <Vx/VxHomokinetic.h>
#include <Vx/VxPrismatic.h>
#include <Vx/VxScrewJoint.h>
#include <Vx/VxSpring.h>
#include <Vx/VxDistanceJoint.h>
#include <Vx/VxBallAndSocket.h>
#include <Vx/VxAngular2Position3.h>
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxSolverParameters.h>
#include <Vx/VxMaterialTable.h>
#include <Vx/VxContactMaterial.h>
#include <Vx/VxBoundingSphere.h>

#include <CableSystems/CableSystemsICD.h>
#include <CableSystems/DynamicsICD.h>
#include <CableSystems/GraphicsICD.h>
#include <Vx/VxConnectionFactory.h>
#include "MySolverExtension.h"

#include <cassert>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>

#define PI 3.141592653589793238462643383
#define MAXGRIDSIZE 150

using namespace Vx;
using namespace CableSystems;
using namespace CableSystems::DynamicsICD;

typedef VxDistanceJoint MyConstraint;

typedef VxConstraint* (*attachFunction)(VxAssembly* assembly, VxPart *p1, VxPart* p2, VxReal compression, VxReal sphereRadius1, VxReal sphereRadius2, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping, VxReal threadBendingStiffness, VxReal threadBendingDamping, VxReal threadTransverseStiffness, VxReal threadTransverseDamping);

// Global variables accessible to all functions
VxPart *partMatrix[MAXGRIDSIZE][MAXGRIDSIZE];
VxPart* cornerMasses[4];
VxPart* chaserPart;
VxPart* targetPart;
VxPart* winchPart;
VxPart* groundPart;
VxConstraint * hinge;

//VxConstraint *constraintMatrix[MAXGRIDSIZE][MAXGRIDSIZE][2]; // joints stemming from part to the north and east direction.
VxConstraint *allConstraintsVector[2*MAXGRIDSIZE*(MAXGRIDSIZE-1)+4]; // all joints
int const NetSize = 21; // nodi su un lato
int gNetSize = NetSize;
VxReal totalMass = 0;
static const VxReal timeStep = 0.01;

VxGraphics::ShapeGenerator *mShapeGenerator;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// This class handles the keyboard key presses
// 
class MyKeySub : public VxSim::IKeyboard, public VxSim::IExtension
{
public:
    MyKeySub(VxSim::VxExtension* iProxy) 
        : VxSim::IKeyboard(iProxy)
        , VxSim::IExtension(iProxy)
    {
        addKeyDescription(' ', "Drop the corner boxes");
    }

    // Handler for key
    virtual void onKeyPressed(int command)
    {
        switch(command)
        {
        case ' ': // space bar 
            partMatrix[0][0] ->freeze(false); 
            partMatrix[0][NetSize-1] ->freeze(false); 
            partMatrix[NetSize-1][0] ->freeze(false); 
            partMatrix[NetSize-1][NetSize-1] ->freeze(false); 
            /*int indexCenterPart;
            if(NetSize%2 != 0) // NetSize is odd
            {
            indexCenterPart = (int)(NetSize-1)/2 ;
            }
            partMatrix[indexCenterPart][indexCenterPart] ->freeze(false);
            *///hinge->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlFree);
            //hinge->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, -2.0);
            // for (int i=0; i<4; ++i)
            //     cornerMasses[i]->freeze(false); // release the corner masses.
            break;
        }
    }
};


// Constraint creation function: create a distance constraint, which has no effect until distance >= the specified distance
VxConstraint* attachDistance(VxAssembly* assembly, VxPart *p1, VxPart* p2, VxReal MeshLength, VxReal sphereRadius1, VxReal sphereRadius2, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping, VxReal threadBendingStiffness, VxReal threadBendingDamping, VxReal threadTransverseStiffness, VxReal threadTransverseDamping)
{
    VxDistanceJoint* d = new VxDistanceJoint(p1,p2,MaxDistance); // keep distance smaller than MeshLength
    d->setRelaxationStiffness(VxDistanceJoint::kConstraintD, threadStiffness);  // 100000 219910
    d->setRelaxationDamping(VxDistanceJoint::kConstraintD, 5000.0 * threadDamping);		//10000 21991
    assembly->addConstraint(d);
    return d;
}

/*
Create a net made out of sphere nodes and cables between them simulated by distance or other joints in the xy plane, at z=frameHeight

Parameters:
attach: the function to use for creating constraints
int ucount, vcount:  number of nodes in x and y
VxReal stepX, stepY: length of cable
VxReal meshMass: total mesh mass
VxReal frameHeight: height of net
VxReal sphereRadius: 
VxReal compression:

*/

void createNetSpheres(attachFunction attach, VxAssembly* assembly, int ucount, int vcount, VxReal stepX, VxReal stepY, VxReal sphereMasses[][NetSize], VxReal Z, VxReal sphereRadii[][NetSize], VxReal MeshLength, VxReal threadStiffness, VxReal threadDamping, VxReal threadBendingStiffness, VxReal threadBendingDamping, VxReal threadTransverseStiffness, VxReal threadTransverseDamping, VxMaterial* MyNetMaterial00, VxMaterial* MyNetMaterial01, VxMaterial* MyNetMaterial11, VxReal indexCenterPart, int nbCables)
{
    assert(ucount < MAXGRIDSIZE && vcount < MAXGRIDSIZE);
    //int i,j,k;
    // VxReal meshNodeMass = meshMass / (ucount*vcount);
    // VxVector3 lowerLeftCorner(- stepX * (ucount-1) / 2 * compression , -stepY * (vcount-1) / 2 , Z);
    VxVector3 lowerLeftCorner(0 , 0 , Z);

    for (int i=0; i<ucount; i++)
    {
        for (int j=0; j<vcount; j++)
        {
            VxGeometry* sphere = new VxSphere(sphereRadii[i][j]);
            VxPart *p = new VxPart(sphereMasses[i][j]);
            VxCollisionGeometry *cg = nullptr;

            if ( ( (i==0)&&(j==0) ) || ( (i==NetSize-1)&&(j==0) ) || ( (i==0)&&(j==NetSize-1) ) || ( (i==NetSize-1)&&(j==NetSize-1) )  )
                cg = new VxCollisionGeometry(sphere,MyNetMaterial00); // reuse the same sphere geometry
            else if ( ( (i>0) && (i<NetSize-1) && (j==0) ) || ( (i>0) && (i<NetSize-1) && (j==NetSize-1) ) || ( (j>0) && (j<NetSize-1) && (i==0) ) || ( (j>0) && (j<NetSize-1) && (i==NetSize-1) )  )
                cg = new VxCollisionGeometry(sphere,MyNetMaterial01);
            else
                cg = new VxCollisionGeometry(sphere,MyNetMaterial11);

            p->addCollisionGeometry(cg);
            partMatrix[i][j] = p;
            VxReal x_curr = stepX*i;
            VxReal y_curr = stepY*j;
            VxReal z_curr = 2.958 - 0.4253*x_curr - 0.4249*y_curr + 0.1969*pow(x_curr,2.0) + 0.1967*pow(y_curr,2.0); // catenary cables as initial condition
            // VxVector3 pos = lowerLeftCorner + VxVector3(stepX*i, stepY*j, z_curr);
            VxVector3 pos = VxVector3(x_curr, y_curr, z_curr);
            p->setPosition(pos);

            if ( ( (i==0)&&(j==0) ) || ( (i==NetSize-1)&&(j==0) ) || ( (i==0)&&(j==NetSize-1) ) || ( (i==NetSize-1)&&(j==NetSize-1) ) ) 
                p ->freeze(true); 

            if ( (nbCables >= 1) && ( (i==indexCenterPart)&&(j==indexCenterPart) )    )
                p ->freeze(true); 

            assembly->addPart(p);
        }        
    }

    // create attachments directly in allConstraintsVector
    int counter = 0;
    for (int j=0; j<=NetSize-1; j++)
    {
        for (int i=0; i<=NetSize-1; i++)
        {
            if (i < NetSize-1)
            {
                allConstraintsVector[counter] = attach(assembly, partMatrix[i][j], partMatrix[i+1][j], MeshLength, sphereRadii[i][j], sphereRadii[i+1][j], MeshLength, threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, threadTransverseStiffness, threadTransverseDamping);
                counter = counter+1;
            }
            if (j < NetSize-1)
            {
                allConstraintsVector[counter] = attach(assembly, partMatrix[i][j], partMatrix[i][j+1], MeshLength, sphereRadii[i][j], sphereRadii[i][j+1], MeshLength, threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, threadTransverseStiffness, threadTransverseDamping);
                counter = counter+1;
            }
        }
    }
}

void addCornerMasses(attachFunction attach, VxAssembly* assembly, VxReal cornerThreadLength, VxReal compression, VxReal NetSideLength, VxReal Z, VxReal frameSideLength, VxReal BulletMass, VxReal netDensity, VxReal alDensity, VxReal netRadius, VxReal sphereRadius, VxReal CornerThreadStiffness, VxReal CornerThreadDamping, VxReal CornerThreadBendingStiffness, VxReal CornerThreadBendingDamping, VxReal CornerThreadTransverseStiffness, VxReal CornerThreadTransverseDamping, FILE* FileLog, VxMaterial* MyCmMaterial)
{
    // here sphereRadius is the radius of the mass at the corner of the net (sphereRadii[0][0])
    //
    VxReal cornerMassInitialHeight = Z - cornerThreadLength; 
    VxReal CornerMass;
    VxReal cornerMassRadius;
    for (int i=0; i<4; i++)
    {
        CornerMass = BulletMass + netDensity * cornerThreadLength * PI * pow(netRadius,2);
        totalMass = totalMass + CornerMass;
        cornerMasses[i] = new VxPart(CornerMass);	// defined global. type VxPart*
        cornerMassRadius = pow(0.75 * CornerMass / PI / alDensity , 1/3.); // x 10 for visualization only; 
        cornerMasses[i]->addCollisionGeometry(new VxCollisionGeometry(new VxSphere(cornerMassRadius),MyCmMaterial));
        fprintf(FileLog, "\nThe mass of the corner mass of index  %i is %g kg \n", i, CornerMass);
        fprintf(FileLog, "\nThe radius of the corner mass of index  %i is %g m \n", i, cornerMassRadius);
        ////std::cout << "The mass of the corner mass of index " << i << " is " << CornerMass << " kg" << '\n'; 
        ////std::cout << "The radius of the corner mass of index " << i << " is " << cornerMassRadius << " m" << '\n';
    }

    cornerMasses[0]->setPosition(0, 0, cornerMassInitialHeight);
    cornerMasses[1]->setPosition(frameSideLength, 0, cornerMassInitialHeight);
    cornerMasses[2]->setPosition(0, frameSideLength, cornerMassInitialHeight);
    cornerMasses[3]->setPosition(frameSideLength, frameSideLength, cornerMassInitialHeight);

    for (int i=0; i<4; i++)
    {
        //std::cout << "The position of the corner mass of index " << i  << " is " << cornerMasses[i]->getPosition() << " m" << '\n'; 
        //cornerMasses[i]->freeze(true);
        assembly->addPart(cornerMasses[i]);
    }

    fprintf(FileLog, "\nThe length of the corner threads  is %g m \n", cornerThreadLength);
    fprintf(FileLog, "\nThe stiffness of the corner threads  is %g N/m \n", CornerThreadStiffness);
    fprintf(FileLog, "\nThe damping of the corner threads  is %g kg Ns/m", CornerThreadDamping);

    allConstraintsVector[2*NetSize*(NetSize-1) ] = attach( assembly, cornerMasses[0], partMatrix[0][0], compression, cornerMassRadius, sphereRadius, cornerThreadLength, CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, CornerThreadTransverseStiffness, CornerThreadTransverseDamping);
    allConstraintsVector[2*NetSize*(NetSize-1) + 1 ] =  attach( assembly, cornerMasses[1], partMatrix[NetSize-1][0], compression, cornerMassRadius, sphereRadius, cornerThreadLength, CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, CornerThreadTransverseStiffness, CornerThreadTransverseDamping); //NetSize-1 is the index of the last part in that row
    allConstraintsVector[2*NetSize*(NetSize-1) + 2 ] = attach( assembly, cornerMasses[2], partMatrix[0][NetSize-1], compression, cornerMassRadius, sphereRadius, cornerThreadLength, CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, CornerThreadTransverseStiffness, CornerThreadTransverseDamping);
    allConstraintsVector[2*NetSize*(NetSize-1) + 3 ] = attach( assembly, cornerMasses[3], partMatrix[NetSize-1][NetSize-1], compression, cornerMassRadius, sphereRadius, cornerThreadLength, CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, CornerThreadTransverseStiffness, CornerThreadTransverseDamping);

    FILE* outP=0;
    outP = fopen("MappingCornersToAllConstraintsVector.txt", "w");
    fprintf(outP, "%i \n ", 2*NetSize*(NetSize-1) ); 
    fprintf(outP, "%i \n ", 2*NetSize*(NetSize-1)+1 ); 
    fprintf(outP, "%i \n ", 2*NetSize*(NetSize-1)+2 ); 
    fprintf(outP, "%i \n ", 2*NetSize*(NetSize-1)+3 ); 
    fclose(outP);
}

void displayThreadsAndCornerMasses(int ucount, int vcount, int indexCenterPart, VxReal winchRadius, VxReal chaserEdgeLength, VxReal targetEdgeLength, int CMpresence, int tetherPresence, int blimpPresence)
{
    // display threads
    static Vx::VxColor threadColor = Vx::VxColor(1.0,1.0,0.0,1.0); // (red, green, blue, transparency)
    static Vx::VxColor tetherColor1 = Vx::VxColor(1.0,0.0,0.0,1.0); // (red, green, blue, transparency)
    static Vx::VxColor tetherColor2 = Vx::VxColor(0.0,1.0,0.0,1.0); // (red, green, blue, transparency)
    static Vx::VxColor boxColor = Vx::VxColor(1.0,1.0,1.0,1.0); // (red, green, blue, transparency)
    static std::vector<Vx::VxVector3> row;
    static std::vector<Vx::VxVector3> col;
    int i,j;

    row.resize(vcount);
    col.resize(ucount);

    // draw net threads
    for (i=0; i<ucount; i++)
    {
        for (j=0; j<vcount; j++)
        {
            row[j] = partMatrix[i][j]->getPosition();
        }
        mShapeGenerator->drawLines(row, 1.0 /* width */, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
    }

    for (j=0; j<vcount; j++)
    {
        for (i=0; i<ucount; i++)
        {
            col[i] = partMatrix[i][j]->getPosition();
        }
        mShapeGenerator->drawLines(col, 1.25 /* width */, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
    }

    Vx::VxTransform pointTransform; 
    if (CMpresence == 1)
    {
        // display corner threads
        mShapeGenerator->drawLine(cornerMasses[0]->getPosition(), partMatrix[0][0]->getPosition(), 1.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
        mShapeGenerator->drawLine(cornerMasses[1]->getPosition(), partMatrix[NetSize-1][0]->getPosition(), 1.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
        mShapeGenerator->drawLine(cornerMasses[2]->getPosition(), partMatrix[0][NetSize-1]->getPosition(), 1.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
        mShapeGenerator->drawLine(cornerMasses[3]->getPosition(), partMatrix[NetSize-1][NetSize-1]->getPosition(), 1.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);

        //display corner masses	
        VxReal CMradius;
        for (i=0; i<4; i++)
        {
            pointTransform.makeTranslation(cornerMasses[i]->getPosition());
            CMradius = cornerMasses[i]->getBoundingSphere().radius();
            pointTransform[0][0] = CMradius*2; pointTransform[1][1] = CMradius*2; pointTransform[2][2] = CMradius*2; 
            mShapeGenerator->drawSphere(VxMath::Matrix44(pointTransform.m), VxGraphics::ShapeGenerator::DEFAULT_PASS, threadColor);
        }
    }

    //display chaser spacecraft
    if (tetherPresence == 1)
    {
        pointTransform.makeTranslation(chaserPart->getPosition());
        ////std::cout << "Position of cube is " << chaserPart.getPosition() << '\n'; 
        pointTransform[0][0] = chaserEdgeLength; pointTransform[1][1] = chaserEdgeLength; pointTransform[2][2] = chaserEdgeLength;
        mShapeGenerator->drawBox(VxMath::Matrix44(pointTransform.m), VxGraphics::ShapeGenerator::WIREFRAME_PASS, boxColor);
    }

    //display target spacecraft
    if (blimpPresence == 1)
    {
        //pointTransform.makeFromTranslationAndRotationEulerAngles(targetPart->getPosition(), 0.0, PI/4.0, PI/4.0); 
        pointTransform = targetPart->getTransform();
        //std::cout << pointTransform[0][0] <<","  << pointTransform[0][1] <<","  << pointTransform[0][2] <<"," << pointTransform[0][3] <<",\n"  << pointTransform[1][0] <<","  << pointTransform[1][1] <<","  << pointTransform[1][2] <<","<<  pointTransform[1][3] <<",\n"  << pointTransform[2][0] <<","  << pointTransform[2][1] <<","  << pointTransform[2][2] <<"," << pointTransform[2][3] <<","<< pointTransform[3][0] <<","  << pointTransform[3][1] <<","  << pointTransform[3][2] <<","<<  pointTransform[3][3] << "\n\n";
        ////std::cout << "Position of cube is " << chaserPart.getPosition() << '\n'; 
        // scaling for blimp side length has to be done on translation and the 1 (found out by trial and error)
        pointTransform[3][0] = pointTransform[3][0]/targetEdgeLength; pointTransform[3][1] = pointTransform[3][1]/targetEdgeLength; pointTransform[3][2] = pointTransform[3][2]/targetEdgeLength;
        pointTransform[3][3] = pointTransform[3][3]/targetEdgeLength;
        //std::cout << pointTransform[0][0] <<","  << pointTransform[0][1] <<","  << pointTransform[0][2] <<"," << pointTransform[0][3] <<",\n"  << pointTransform[1][0] <<","  << pointTransform[1][1] <<","  << pointTransform[1][2] <<","<<  pointTransform[1][3] <<",\n"  << pointTransform[2][0] <<","  << pointTransform[2][1] <<","  << pointTransform[2][2] <<"," << pointTransform[2][3] <<","<< pointTransform[3][0] <<","  << pointTransform[3][1] <<","  << pointTransform[3][2] <<","<<  pointTransform[3][3] << "\n\n";
        //system("pause");
        mShapeGenerator->drawBox(VxMath::Matrix44(pointTransform.m), VxGraphics::ShapeGenerator::DEFAULT_PASS, boxColor);
    }

    // display ground
    /*pointTransform.makeTranslation(groundPart->getPosition());
    pointTransform[0][0] = 10; pointTransform[1][1] = 10; pointTransform[2][2] = 0.5;
    mShapeGenerator->drawBox(pointTransform, VxGraphics::ShapeGenerator::DEFAULT_PASS, boxColor);*/

    // display tether
    if (tetherPresence == 1)
    {
        // display cable 1
        mShapeGenerator->drawLine(winchPart->getPosition()+Vx::VxVector3(0, winchRadius, 0), partMatrix[indexCenterPart][indexCenterPart]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor1);	
        mShapeGenerator->drawLine(partMatrix[indexCenterPart][indexCenterPart]->getPosition(), partMatrix[0][indexCenterPart]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor1);
        mShapeGenerator->drawLine(partMatrix[0][indexCenterPart]->getPosition(), partMatrix[0][1]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor1);
        mShapeGenerator->drawLine(partMatrix[0][1]->getPosition(), partMatrix[1][0]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor1);
        mShapeGenerator->drawLine(partMatrix[1][0]->getPosition(), partMatrix[indexCenterPart][0]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor1);
        mShapeGenerator->drawLine(partMatrix[indexCenterPart][0]->getPosition(), partMatrix[NetSize-2][0]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor1);
        mShapeGenerator->drawLine(partMatrix[NetSize-2][0]->getPosition(), partMatrix[NetSize-1][1]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor1);
        mShapeGenerator->drawLine(partMatrix[NetSize-1][1]->getPosition(), partMatrix[NetSize-1][indexCenterPart]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor1);

        // display cable 2
        mShapeGenerator->drawLine(winchPart->getPosition()+Vx::VxVector3(0, winchRadius, 0), partMatrix[indexCenterPart][indexCenterPart]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor2);	
        mShapeGenerator->drawLine(partMatrix[indexCenterPart][indexCenterPart]->getPosition(), partMatrix[NetSize-1][indexCenterPart]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor2);
        mShapeGenerator->drawLine(partMatrix[NetSize-1][indexCenterPart]->getPosition(), partMatrix[NetSize-1][NetSize-2]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor2);
        mShapeGenerator->drawLine(partMatrix[NetSize-1][NetSize-2]->getPosition(), partMatrix[NetSize-2][NetSize-1]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor2);
        mShapeGenerator->drawLine(partMatrix[NetSize-2][NetSize-1]->getPosition(), partMatrix[indexCenterPart][NetSize-1]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor2);
        mShapeGenerator->drawLine(partMatrix[indexCenterPart][NetSize-1]->getPosition(), partMatrix[1][NetSize-1]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor2);
        mShapeGenerator->drawLine(partMatrix[1][NetSize-1]->getPosition(), partMatrix[0][NetSize-2]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor2);
        mShapeGenerator->drawLine(partMatrix[0][NetSize-2]->getPosition(), partMatrix[0][indexCenterPart]->getPosition(), 2.0, VxGraphics::ShapeGenerator::DEFAULT_PASS, tetherColor2);
    }
}

void modifyMaterialTable(Vx::VxMaterialTable* myTable)
{
    std::cout << "\n You have to modify the material table! \n";
    // Create Spacecraft Material (ID=1): parameters are changed in ContactMaterials later
    Vx::VxMaterial* SpacecraftMaterial = myTable->registerMaterial("MyGroundMaterial");
    // normal direction
    SpacecraftMaterial->setCompliance(1.0e-6f); 
    SpacecraftMaterial->setDamping(1.0e005f);
    // tangential direction
    SpacecraftMaterial->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    SpacecraftMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.45*4.0/5.0); //Al-Al
    SpacecraftMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25
    SpacecraftMaterial->setSlip(VxMaterialBase::kFrictionAxisLinearPrimary, 1.0);		

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
    NetMaterial00->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.2*4.0/5.0); //Nylon-Nylon
    NetMaterial00->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25
    NetMaterial00->setSlip(VxMaterialBase::kFrictionAxisLinearPrimary, 0.000001);	

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

    //// Create Corner Masses Material (ID=6):  parameters are changed in ContactMaterials later
    //Vx::VxMaterial* CmMaterial = myTable->registerMaterial("MyBlimpMaterial");
    //// normal direction
    //CmMaterial->setCompliance(1.0e-6f); 
    //   CmMaterial->setDamping(1.0e005f);	
    //// tangential direction
    //CmMaterial->setFrictionModel(VxMaterialBase::kFrictionAxisLinear, VxMaterialBase::kFrictionModelScaledBox);
    //   CmMaterial->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 0.45*4.0/5.0); //Al-Al
    //CmMaterial->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25
    //CmMaterial->setSlip(VxMaterialBase::kFrictionAxisLinearPrimary, 0.000001);	


    //if ( (NetSize == 21) && ( NetSideLength==20.0 ) && (netRadius==0.0005) )
    //{
    // Computed from Hertz theory, according to contact geometry
    VxContactMaterial *cm = myTable->getContactMaterial(SpacecraftMaterial, CmMaterial);
    cm->setCompliance(1000.0/1816300);
    cm->setDamping(1396);
    cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 1.45*4.0/5.0); //Al-Al
    cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

    cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial00);
    cm->setCompliance(1000.0/1816300);
    cm->setDamping(70);
    cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 100.0*4.0/5.0); // Al-Nylon
    cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

    cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial01);
    cm->setCompliance(1000.0/1816300);
    cm->setDamping(53);
    cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear,100.0*4.0/5.0); // Al-Nylon
    cm->setStaticFrictionScale(VxMaterialBase::kFrictionAxisLinear,5.0/4.0);   //=1.25

    cm = myTable->getContactMaterial(SpacecraftMaterial, NetMaterial11);
    cm->setCompliance(1000.0/1816300);
    cm->setDamping(62);
    cm->setFrictionCoefficient(VxMaterialBase::kFrictionAxisLinear, 100.0*4.0/5.0); // Al-Nylon
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
    //	 Vx::VxFatalError(0, "Contact materials cannot be computed.");
}

VxPart* addBoxChaser(VxAssembly* assembly, VxReal chaserMass, VxReal chaserEdgeLength, VxReal NetSideLength, VxReal compression, VxReal Zbox, VxReal winchRadius, VxMaterial* MyAluminumMaterial)
{
    // Create a dynamic part that contains a box as collision geometry 
    VxPart* tempChaserPart = new VxPart(chaserMass);
    tempChaserPart->addCollisionGeometry(new VxCollisionGeometry(new VxBox(chaserEdgeLength, chaserEdgeLength, chaserEdgeLength),MyAluminumMaterial));
    //VxMaterial* BoxMaterial = chaserPart->getCollisionGeometries()->getMaterial();
    //chaserPart->setControl(Vx::VxPart::kControlStatic);
    tempChaserPart->setPosition(NetSideLength*compression/2, NetSideLength*compression/2-winchRadius, Zbox); // centrata wrt net
    assembly->addPart(tempChaserPart);
    //std::cout << "Position of cube is " << tempChaserPart->getPosition() << '\n'; 

    return tempChaserPart;
}

VxPart* addBoxTarget(VxAssembly* assembly, VxReal TargetMass, VxReal TargetEdgeLength, VxReal NetSideLength, VxReal compression, VxReal Zbox, VxReal winchRadius, VxMaterial* MyTargetMaterial)
{
    // Create a dynamic part that contains a box as collision geometry 
    VxPart* tempTargetPart = new VxPart(TargetMass);
    tempTargetPart->addCollisionGeometry(new VxCollisionGeometry(new VxBox(TargetEdgeLength, TargetEdgeLength, TargetEdgeLength),MyTargetMaterial));
    //VxMaterial* BoxMaterial = TargetPart->getCollisionGeometries()->getMaterial();
    tempTargetPart->setPosition(NetSideLength*compression/2, NetSideLength*compression/2, Zbox); // centrata wrt net
    std::cout << "Position of cube is " << tempTargetPart->getPosition() << '\n'; 
    tempTargetPart->setOrientationEulerAngles( 0.0, PI/4.0, PI/4.0);
    //std::cout << "Position of cube is " << tempTargetPart->getPosition() << '\n'; 
    VxReal3 buoyancyForce = {0,0,9.81*TargetMass};
    tempTargetPart->addForce(buoyancyForce);
    assembly->addPart(tempTargetPart);


    return tempTargetPart;
}


// add ground
VxPart* addGround(VxAssembly* assembly, VxReal NetSideLength, VxReal compression, VxMaterial* MyBoxMaterial)
{
    // Create a static part that contains a box as collision geometry 
    VxPart* staticPart = new VxPart();
    VxReal groundPartWidth = 0.5;
    staticPart->addCollisionGeometry(new VxCollisionGeometry(new VxBox(10, 10, groundPartWidth),MyBoxMaterial));
    staticPart->setControl(Vx::VxPart::kControlStatic);
    staticPart->setPosition(NetSideLength*compression/2, NetSideLength*compression/2, groundPartWidth/2); // centrata wrt net
    assembly->addPart(staticPart);

    return staticPart;
}

VxPart* addWinch(VxAssembly* assembly, VxReal winchRadius, VxReal NetSideLength, VxReal compression, VxReal Zbox, VxReal chaserEdgeLength, VxMaterial* MyAluminumMaterial)
{
    VxPart* tempWinchPart = new VxPart();    
    //tempWinchPart->setName(sWinchName.c_str());    
    tempWinchPart->setControl(Vx::VxPart::kControlDynamic);    
    tempWinchPart->setPosition(NetSideLength*compression/2, NetSideLength*compression/2-winchRadius, Zbox-chaserEdgeLength/2.0);
    VxTransform tm(VxVector3(0,0,0), VxEulerAngles(0.0, VX_HALF_PI, 0.0));
    VxCollisionGeometry* drum = new VxCollisionGeometry(new VxCylinder(winchRadius, 0.02));
    // The long axis of the cylinder is along its local z axis.// Set it parallel to the x axis of the base.    
    drum->setTransformRelative( tm );    
    tempWinchPart->addCollisionGeometry(drum);
    assembly->addPart(tempWinchPart);
    return tempWinchPart;
}

// Create scene with parts linked with various constraints.
Vx::VxAssembly* createSceneSpheres(VxReal netMass, VxReal NetSideLength, VxReal MeshLength, VxReal netRadius, VxReal netDensity, VxReal netYoungModulus, VxReal compression, int indexCenterPart, VxReal winchRadius, int nbCables, VxReal chaserEdgeLength, VxReal attachmentToWinch,  VxReal targetEdgeLength, VxReal targetMass, int CMpresence, VxReal bulletMass, VxReal cornerThreadLength, VxReal metalRingMass, int blimpPresence, VxReal frameHeight, VxReal blimpBottomHeight, VxReal winchingSpeed, VxReal frameSideLength)
{
    FILE* FileLog=0;
    FileLog = fopen("FileLog.txt", "w");

    // Create an assembly and add it to the assembly.
    Vx::VxAssembly* assembly = new Vx::VxAssembly();

    // Change material table
    Vx::VxMaterialTable* myTable = VxFrame::instance()->getUniverse(0)->getMaterialTable(); // gets material table 	
    modifyMaterialTable(myTable);	
    const char* MaterialTableForFile="MaterialTable" ; 
    myTable->dump (MaterialTableForFile); // print material table to file

    // Properties CMs
    VxReal alDensity = 2700;

    // Parameters chaser
    VxReal chaserZ = frameHeight + attachmentToWinch + chaserEdgeLength/2.0;
    VxReal chaserMass = 1000;
    //VxReal scaleSpacecraft = 6;  

    // Parameters target
    VxReal targetDiagonalLength = pow(3.0,0.5)*targetEdgeLength;
    VxReal targetZ = blimpBottomHeight + targetDiagonalLength/2.0;   //position: [ 0 0 targetZ] set in function addBoxTarget; rotation: [0 45deg 45 deg] fixed and set in function too;
    std::cout << "\n Check the position and orientation of the target \n ";

    // Properties of tether
    std::cout << "\n You must define the properties of the cable properly \n ";
    //system("pause");
    VxReal cableRadius = 0.0013; 
    VxReal cableDensity = netDensity;
    VxReal cableLength = 6;
    VxReal cableYoungModulus = netYoungModulus;
    VxReal wn1AxialCable = PI/2.0/cableLength * pow( cableYoungModulus/cableDensity, 0.5);

    // Net stiffness and damping evaluation
    VxReal csiAxial = 0.106;				// damping ratio, from Salvi
    //VxReal csiAxial = 10.0;
    VxReal csiBending = 0.014;				// damping ratio for bending, from Salvi

    VxReal wn1Axial;
    std::cout << "\n You must re-compute the natural frequencies with AML tests/main_computeNaturalFrequencies.m if you changed density \n ";
    if ( (netMass == 0.49) && (NetSize==21) && ( NetSideLength==2.0) && (CMpresence==0) )
        wn1Axial = 1115.706993711777;			// first natural frequency, as found with Matlab
    else if ( (netMass == 0.49) && (NetSize==21) && ( NetSideLength==2.1) && (CMpresence==0) )
        wn1Axial = 1062.57808924942;
    else if ( (netMass == 0.49) && (NetSize==31) && ( NetSideLength==2.1) && (CMpresence==0) )
        wn1Axial = 916.9888984059663;
    else if ( (netMass == 0.49) && (NetSize==41) && ( NetSideLength==2.1) && (CMpresence==0) )
        wn1Axial = 817.5569363722726;
    else if ( (netMass == 0.792) && (NetSize==21) && ( NetSideLength==2.25) && (CMpresence==0) )
        wn1Axial =  1165.255479037686; 	
    else if ( (netMass == 0.792) && (NetSize==41) && ( NetSideLength==2.25) && (CMpresence==0) )
        wn1Axial =  926.9501836570548;
    else if ( (netMass == 0.792) && (NetSize==51) && ( NetSideLength==2.25) && (CMpresence==0) )
        wn1Axial = 850.8685396709821;
    else if ( (netMass == 0.792) && (NetSize==111) && ( NetSideLength==2.25) && (CMpresence==0) )
        wn1Axial =  600.0; // not real
    else
    {
        std::cout << "\n You must re-compute the natural frequencies with AML tests/main_computeNaturalFrequencies.m \n ";
        system("pause");
    }

    // create spheres in the net
    VxReal sphereMasses[NetSize][NetSize];
    for (int i=0; i<NetSize; i++)
    {
        for (int j=0; j<NetSize; j++)
        {
            sphereMasses[i][j] = 4 * netDensity * MeshLength * PI *pow(netRadius,2) / 2;
            if ( ( (i==0)&&(j==0) ) || ( (i==NetSize-1)&&(j==0) ) || ( (i==0)&&(j==NetSize-1) ) || ( (i==NetSize-1)&&(j==NetSize-1) )  )
            {
                if (CMpresence == 0)
                    sphereMasses[i][j] = metalRingMass + 2 * netDensity * MeshLength * PI * pow(netRadius,2) / 2; 
                else
                    sphereMasses[i][j] = metalRingMass + 2 * netDensity * MeshLength * PI * pow(netRadius,2) / 2 + netDensity * cornerThreadLength * PI * pow(netRadius,2); // assuming yellow thread = net thread 
            }
            else if ( ( (i>0) && (i<NetSize-1) && (j==0) ) || ( (i>0) && (i<NetSize-1) && (j==NetSize-1) ) || ( (j>0) && (j<NetSize-1) && (i==0) ) || ( (j>0) && (j<NetSize-1) && (i==NetSize-1) )  )
                sphereMasses[i][j] = 3 * netDensity * MeshLength * PI * pow(netRadius,2) / 2;

            fprintf(FileLog, "\nThe mass of the sphere of index  %i,%i is %g kg \n", i,j,sphereMasses[i][j]);
            ////std::cout << "The mass of the sphere of index " << i << "," << j << " is " << sphereMasses[i][j] << " kg" << '\n'; 
            totalMass = totalMass + sphereMasses[i][j];
        }
    }

    VxReal sphereRadii[NetSize][NetSize];
    for (int i=0; i<NetSize; i++)  
    {
        for (int j=0; j<NetSize; j++)
        {
            sphereRadii[i][j] = pow((0.75 / PI * (sphereMasses[i][j]) / netDensity),(1/3.)); // x 10 for visualization only
            fprintf(FileLog, "\nThe radius of the sphere of index  %i,%i is %g m \n", i,j,sphereRadii[i][j]);
        }
    }

    // define stiffnesses of the net threads
    std::cout << " \n Check definition of the corner threads stiffness. \n";
    VxReal threadStiffness = netYoungModulus * PI * pow(netRadius,2.00) / MeshLength;
    VxReal CornerThreadStiffness = netYoungModulus * PI * pow(netRadius,2.0) * 2.00 /cornerThreadLength;
    VxReal threadDamping = 2 * csiAxial / wn1Axial * threadStiffness;
    VxReal CornerThreadDamping = 2 * csiAxial / wn1Axial * CornerThreadStiffness;
    VxReal cableStiffness = cableYoungModulus * PI * pow(cableRadius,2.00) / cableLength;	
    VxReal cableDamping = 2 * csiAxial / wn1AxialCable * cableStiffness;

    // the next 8 lines are about variables that are not used with VxDistanceJoints
    VxReal threadBendingStiffness = netYoungModulus * PI / 4.0 * pow(netRadius,4) / MeshLength; // EI/l
    VxReal CornerThreadBendingStiffness = netYoungModulus * PI * pow(netRadius,4) / cornerThreadLength; // EI/l corner threads
    VxReal threadBendingDamping = 2 * csiAxial / wn1Axial * threadBendingStiffness;  // da cambiare
    VxReal CornerThreadBendingDamping = 2 * csiAxial / wn1Axial * CornerThreadBendingStiffness; // da cambiare
    VxReal threadTransverseStiffness = 3.0 * netYoungModulus * PI / 4.0 * pow(netRadius,4) / pow(cornerThreadLength,3) ; // 3EI/l^3
    VxReal CornerThreadTransverseStiffness = 3.0 * netYoungModulus * PI * pow(netRadius,4) / pow(cornerThreadLength,3); // 3EI/l^3 corner threads
    VxReal threadTransverseDamping = 2 * csiAxial / wn1Axial * threadTransverseStiffness; // da cambiare
    VxReal CornerThreadTransverseDamping = 2 * csiAxial / wn1Axial * CornerThreadTransverseStiffness; // da cambiare

    // write in log file
    fprintf(FileLog, "The net side lenght is %g m \n ", NetSideLength);
    fprintf(FileLog, "The mesh lenght is %g m \n The corner thread lenght is %g m \n ", MeshLength, cornerThreadLength);
    //fprintf(FileLog, "Each bullet's mass is %g kg \n ", BulletMass );
    fprintf(FileLog, "The net material Young's modulus is %g Pa \n", netYoungModulus);
    fprintf(FileLog, "The thread's radius is %g m \n ", netRadius);
    fprintf(FileLog, "The bullets material density is %g kg/m3 \n The net's material density is %g kg/m3 \n ", alDensity, netDensity);
    fprintf(FileLog, "\nThe threads' stiffness in axial direction is  %g N/m \n", threadStiffness);
    fprintf(FileLog, "\nThe corner threads' stiffness in axial direction is  %g N/m \n", CornerThreadStiffness);
    fprintf(FileLog, "\nThe threads' damping in axial direction  is  %g Ns/m \n", threadDamping);
    fprintf(FileLog, "\nThe corner threads' damping in axial direction  is  %g Ns/m \n\n", CornerThreadDamping);
    fprintf(FileLog, "\nThe threads' stiffness in bending (angular) is  %g Nm \n", threadBendingStiffness);
    fprintf(FileLog, "\nThe corner threads' stiffness in bending (angular) is  %g Nm \n", CornerThreadBendingStiffness);
    fprintf(FileLog, "\nThe threads' damping in bending (angular) is  %g Nms \n", threadBendingDamping);
    fprintf(FileLog, "\nThe corner threads' damping in bending (angular) is  %g Nms \n\n", CornerThreadBendingDamping);
    fprintf(FileLog, "\nThe threads' stiffness in transverse direction is  %g N/m \n", threadTransverseStiffness);
    fprintf(FileLog, "\nThe corner threads' stiffness in transverse direction is  %g N/m \n", CornerThreadTransverseStiffness);
    fprintf(FileLog, "\nThe threads' damping in transverse direction is  %g Ns/m \n", threadTransverseDamping);
    fprintf(FileLog, "\nThe corner threads' damping in transverse direction is  %g Ns/m \n\n", CornerThreadTransverseDamping);
    fprintf(FileLog, "\nThe tether's axial stiffness is  %g N/m \n", cableStiffness);
    fprintf(FileLog, "\nThe tether's axial damping is  %g Ns/m \n\n", cableDamping);

    // Create net 
    createNetSpheres(attachDistance, assembly, NetSize, NetSize, MeshLength*compression, MeshLength*compression, sphereMasses, frameHeight, sphereRadii, MeshLength, threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, threadTransverseStiffness, threadTransverseDamping, myTable->getMaterial(3), myTable->getMaterial(4), myTable->getMaterial(5), indexCenterPart, nbCables);

    // Create corner masses
    if (CMpresence == 1)
    {
        addCornerMasses(attachDistance, assembly, cornerThreadLength, compression, NetSideLength, frameHeight, frameSideLength, bulletMass, netDensity, alDensity, netRadius, sphereRadii[0][0],  CornerThreadStiffness, CornerThreadDamping, CornerThreadBendingStiffness, CornerThreadBendingDamping, CornerThreadTransverseStiffness, CornerThreadTransverseDamping, FileLog,myTable->getMaterial(2));
    }
    fprintf(FileLog, "\nThe total mass is  %1.15f kg \n", totalMass);

    // Create box for s/c
    if (nbCables >= 1)
    {
        chaserPart = addBoxChaser(assembly, chaserMass, chaserEdgeLength, NetSideLength, compression, chaserZ, winchRadius, myTable->getMaterial(0));	
        chaserPart->freeze(true);
    }

    // Create box to be captured
    if (blimpPresence == 1)
    {
        //std::cout << "i am creating the blimp \n";
        //system("pause");
        targetPart = addBoxTarget(assembly, targetMass, targetEdgeLength, NetSideLength, compression, targetZ, winchRadius, myTable->getMaterial(0));	
        targetPart->freeze(false);
    }

    // Create ground
    //groundPart = addGround(assembly, NetSideLength, compression, myTable->getMaterial(1));	

    // Create winch
    if (nbCables >= 1)
    {
        // Create winch in s/c
        winchPart = addWinch(assembly, winchRadius, NetSideLength, compression, chaserZ, chaserEdgeLength, myTable->getMaterial(0));

        // Create constraint between chaserPart and winchPart: this constraint is motorized to enable the spooling of the cable.   
        const VxVector3 axis(1.0, 0.0, 0.0);
        VxConstraint * mHingeForWinch = new VxHinge(chaserPart, winchPart, winchPart->getPosition(), axis);    

        // Locked
        mHingeForWinch->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlLocked);    

        hinge = mHingeForWinch;
        assembly->addConstraint(hinge);
        //std::cout << "Hinge has been created. \n";
    }

    // Disable collision, except with base.
    VxCollisionRule collisionRule(assembly, false);
    assembly->appendCollisionRule(collisionRule);
    VxCollisionRule collisionRule1(chaserPart,winchPart, false);
    assembly->appendCollisionRule(collisionRule1);

    // Change gravity
    VxFrame::instance()->getUniverse(0)->setGravity(0,0,-9.81); // by default we have gravity.

    fclose(FileLog);

    return assembly;
}

VxSim::VxExtension* createExtension(const std::string& name, const VxSim::VxFactoryKey key)
{
    VxPluginSystem::VxPluginManager::instance()->load(name);
    if (!VxPluginSystem::VxPluginManager::instance()->isLoaded(name))
    {
        Vx::VxWarning(0, "Could not load plugin file: %s\n", name.c_str());
    }
    // Create a matlabGlobalExtension to register associated solver
    VxSim::VxExtension* extension = VxSim::VxExtensionFactory::create(key);
    return extension;
}

// This is where the Framework Application is created and the required modules are inserted.
int main(int argc, const char *argv[])
{
#if 1
    // Setup custom solver extension
    std::string pluginPath("../vortextoolkit/plugins");
    std::string pluginPath2("../plugins/bin");
    VxPluginSystem::VxPluginManager::instance()->addPluginSearchDir(pluginPath);
    VxPluginSystem::VxPluginManager::instance()->addPluginSearchDir(pluginPath2);
    const VxSim::VxFactoryKey kExternalSolverExtensionKey(VxSim::VxUuid("{BD0839CC-C30B-4A41-AB32-39DA52224408}"), "Tutorials", "ExternalSolverExtension");
    VxSim::VxSmartInterface<MySolverExtension> externalSolverExtension = createExtension("MySolverExtension", kExternalSolverExtensionKey);

    // solver indices
    //  0 = default Vortex solver
    //  1 = iterative Vortex solver
    //  2 = Kea solver
    //  3 = Judice solver
    //  4 = Kea (fixed) solver
    //  5 = PGS solver
    //  6 = PGS-SM solver
    //  7 = Spook solver
    externalSolverExtension->pSolverIndex = 5;
    externalSolverExtension->pTimestep = timeStep;
    externalSolverExtension->pIterativeSolverMaxIter = 50;
    externalSolverExtension->pIterativeSolverTol = 1e-4;
    externalSolverExtension->pUseScaledBox = false;
    externalSolverExtension->pPGSEpsilon = 1e-9;

#endif

    int CMpresence = 0;
    int tetherPresence = 0;
    int blimpPresence = 0;
    VxReal frameHeight = 2.95; 
    VxReal winchingSpeed = -1000;
    VxReal blimpBottomHeight = -1000;

    // define parameters wanted
    VxReal NetSideLength, netMass;
    NetSideLength = 2.25;
    netMass = 0.792;

    // define parameters common to all experiments
    VxReal chaserEdgeLength = 0.25;
    VxReal targetBuoyancy = 0.062; // data: 61g - 63g
    VxReal bulletMass = 0.033;
    VxReal metalRingMass = 0.009;
    VxReal cornerThreadLength = 0.1;
    VxReal targetEdgeLength = 0.5;
    VxReal frameSideLength = 2.16;
    int NetSizePhysical = 20;
    VxReal netRadiusPhysical = 0.008;
    VxReal netDensity = netMass / (2*NetSizePhysical*NetSideLength*PI*netRadiusPhysical*netRadiusPhysical);
    std::cout << " \n The net density is " << netDensity <<" \n";
    VxReal nylonYoungModulus = 500.0*9.80665*1000000.0; //Pa   from rt-technical-properties-of-synthetic-fibres-oct-08.pdf
    VxReal packingCoeff = 0.15; // Nylon braid: estimated from Cortland's data
    VxReal netYoungModulus = nylonYoungModulus*packingCoeff; //Pa   
    VxReal netRadius = pow( netMass/(2*NetSize*NetSideLength*PI*netDensity) ,0.5);
    std::cout << " \n The net radius is " << netRadius <<" \n";

    // dummy data
    VxReal compression = frameSideLength/NetSideLength;
    VxReal targetMass = 0.2; 
    VxReal winchRadius = 0.0111; 
    VxReal attachmentToWinch = 0.5; 

    // compute useful quantities
    VxReal MeshLength = NetSideLength / (NetSize-1);

    // initialize tether
    int nbCables;
    if (tetherPresence == 1)
        nbCables = 2; 	
    else 
        nbCables = 0; 

    // Find central mass of net, to attach cable later
    int indexCenterPart;
    if(NetSize%2 != 0) // NetSize is odd
    {
        indexCenterPart = (int)(NetSize-1)/2 ;
        ////std::cout << "Index center part is " << indexCenterPart <<'\n';
    }

    clock_t simulationTime;
    simulationTime = clock();

    int returnValue = 0;

    std::string netMassStr, NetSideLengthStr;
    if (netMass == 0.792)
        netMassStr = "_HighMass";
    else if (netMass == 0.49)
        netMassStr = "_LowMass";

    if (NetSideLength == 2.0)
        NetSideLengthStr = "_SmallerNet";
    else if (NetSideLength == 2.1)
        NetSideLengthStr = "_MediumNet";
    else if (NetSideLength == 2.25)
        NetSideLengthStr = "_LargerNet";
    try 
    {
        // Instantiate the Vortex application.
        Vx::VxSmartPtr<VxSim::VxApplication> application = new VxSim::VxApplication;

        // Instantiate a Graphic module using OSG and add it to the application.   
        VxPluginSystem::VxPluginManager::instance()->load("VxGraphicsModuleOSG");
        Vx::VxSmartPtr<VxSim::VxSimulatorModule> graphicsSimulatorModule = VxSim::VxSimulatorModuleFactory::create(VxGraphicsPlugins::GraphicsModuleICD::kModuleFactoryKey);
        application->insertModule(graphicsSimulatorModule.get());

        // Create a default camera for the Graphic module
        VxSim::VxSmartInterface<VxGraphics::ICamera> freeCamera = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::PerspectiveICD::kExtensionFactoryKey);
        freeCamera->lookAt(Vx::VxVector3(NetSideLength*compression/2, NetSideLength*compression/2-6, frameHeight-1), Vx::VxVector3(NetSideLength*compression/2, NetSideLength*compression/2, frameHeight-1), Vx::VxVector3(0.0, 0.0, 1.0));
        application->add(freeCamera);
        //freeCamera->lookAt(Vx::VxVector3(NetSideLength*compression/2, NetSideLength*compression/2, 15), Vx::VxVector3(NetSideLength*compression/2, NetSideLength*compression/2, -10), Vx::VxVector3(0.0, 0.0, 1.0));

        // Set the free camera as the active camera using the interface specialized for the VxGraphicModule
        VxGraphics::GraphicsModule* graphicModule = VxSim::FindInterface<VxGraphics::GraphicsModule>(graphicsSimulatorModule);
        assert( graphicModule != NULL );
        VxGraphics::Context* context = graphicModule->createDefaultWindow();
        context->getViewport(0)->setCamera(freeCamera);

        // get utility class for creating graphic shapes
        mShapeGenerator = &graphicModule->getSceneGraph()->getServices().getShapeGenerator();

        // Instantiate a Framework dynamics module add it to the application.   
        Vx::VxSmartPtr<VxSim::VxSimulatorModule> dynamicsModule = VxSim::VxSimulatorModuleFactory::create( VxSim::VxDynamicsModuleICD::kFactoryKey);
        application->insertModule(dynamicsModule.get());

        // Instantiate the CableSystemDynamics to create cable
        VxPluginSystem::VxPluginManager::instance()->load("CableSystemsDynamics");

        // Add keyboard help display
        VxSim::VxExtension* helpDisplay = VxSim::VxExtensionFactory::create(KeyboardHelper::KeyboardHelpDisplayICD::kFactoryKey);
        application->add(helpDisplay);

        // Add keyboard command display
        VxSim::VxExtension* keyboardNotificationDisplay = VxSim::VxExtensionFactory::create(KeyboardHelper::KeyboardNotificationDisplayICD::kFactoryKey);
        application->add(keyboardNotificationDisplay);

        // Create the constraints and add them to a scene
        Vx::VxAssembly* assemb = createSceneSpheres(netMass, NetSideLength, MeshLength, netRadius, netDensity, netYoungModulus, compression, indexCenterPart, winchRadius, nbCables, chaserEdgeLength, attachmentToWinch, targetEdgeLength, targetMass, CMpresence, bulletMass, cornerThreadLength, metalRingMass, blimpPresence, frameHeight, blimpBottomHeight, winchingSpeed, frameSideLength);

        // Add the scene at start
        //application->add(scene->getMechanism(0));
        VxFrame::instance()->getUniverse(0)->addAssembly(assemb);

        // 
        application->add(externalSolverExtension);

#ifdef USE_OSG   
        // Instantiate the DynamicsVisualizer if you want to view the physics associated with various parts that have no graphics.   
        VxPluginSystem::VxPluginManager::instance()->load("DynamicsVisualizer");
        VxSim::VxExtension* dynamicsVisualizer = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::DynamicsVisualizerICD::kExtensionFactoryKey);
        dynamicsVisualizer->getInput(VxGraphicsPlugins::DynamicsVisualizerICD::kDisplayCollisionGeometry)->setValue(true);
        application->add(dynamicsVisualizer);
#endif	


        //VxSim::VxExtension* cableDynamicsVisualizer = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::DynamicsVisualizerICD::kExtensionFactoryKey);
        //dynamicsVisualizer->getInput(VxGraphicsPlugins::DynamicsVisualizerICD::kDisplayCollisionGeometry)->setValue(true);

        //VxSim::VxExtension* cableGraphicsExt = VxSim::VxExtensionFactory::create(CableSystemsICD::Extensions::kDynamicsKey);
        //application->add(cableGraphicsExt);


        // Decide solver
        VxSolverParameters *solverPar =  VxFrame::instance()->getUniverse(0)->getSolverParameters(0);
        unsigned int solverType = solverPar->getConstraintSolver();   
        VxFrame::instance()->setTimeStep(timeStep); 
        //solverPar->setConstraintIterativeSolverMaxIteration(25);
        //solverPar->setConstraintIterativeSolverTolerance(1e-5);

        VxReal finalTime = 120.0; 
        int countSteps = 0; 
        VxReal currentTime = 0;

        int ICflag = 0;
        VxReal minEn = 1000;
        VxReal minCenterVel = 1000;

        while((application->update()) && (currentTime<finalTime))
        {    
            countSteps++;
            std::cout << "time: " << currentTime  << std::endl;

            if (blimpPresence==1)
            {
                VxVector3 targetVel = targetPart->getLinearVelocity();
                if ( targetVel.z()>=0 ) 
                { 
                    VxReal3 buoyancyForce = {0,0,9.81*targetMass}; 
                    targetPart->addForce(buoyancyForce);
                }
                else 
                { 
                    VxReal3 buoyancyForce = {0,0,9.81*(targetMass-targetBuoyancy)} ;
                    targetPart->addForce(buoyancyForce);					
                }
            }

            // Function to control the winch
            if (tetherPresence==1)
            {
                // spool-in 	
                /*
                if (currentTime>=0.0)  // change with start time of winching
                {
                std::cout >> " \n did you make sure that the winching time is correct? \n";
                system("pause");
                hinge->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlMotorized);   
                VxReal winchSpoolingVelocity = -winchingSpeed;  // (m/s): length of tether released in 1 s. Negative for spool-in, positive for spool-out
                VxReal winchAngularVelocity = winchSpoolingVelocity / winchRadius;
                hinge->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, winchAngularVelocity);    
                }
                */
                /*
                VxVector3 pos = partMatrix[indexCenterPart][indexCenterPart]->getPosition();
                std::cout << "time: " << currentTime << std::endl;
                std::cout << "Length cable:  " << winchPart->getPosition().z()-pos.z() << std::endl;
                std::cout << "Hinge is locked?  " << hinge->isLocked(VxHinge::kCoordinateLinear) << std::endl;
                std::cout << "Hinge lock pos:  " << hinge->getLockPosition(VxHinge::kCoordinateLinear) << std::endl;
                std::cout << "Hinge current pos:  " << hinge->getCoordinateCurrentPosition(VxHinge::kCoordinateLinear) << std::endl;
                std::cout << "Hinge current ang pos:  " << hinge->getCoordinateCurrentPosition(VxHinge::kAngularCoordinate) << std::endl;
                */
            }
            /*
            // Save cable outputs
            if (tetherPresence==1)
            {
            if (cableExtension.get())
            {
            //VxReal cableID = cableExtension->getOutput(CableSystems::DynamicsICD::kCablesID)->toReal();
            VxReal cableLength = cableExtension->getOutput(CableSystems::DynamicsICD::kTotalLengthID)->toReal();
            VxReal cableElongation = cableExtension->getOutput(CableSystems::DynamicsICD::kElongationID)->toReal();
            VxReal cableMaxTension = cableExtension->getOutput(CableSystems::DynamicsICD::kMaxTensionID)->toReal();
            //std::cout << "Cable: " << cableExtension.getName() << std::endl;
            //std::cout << "Cable ID: " << cableID  << std::endl;
            //std::cout << "Length: " << cableLength  << std::endl;
            //std::cout << "Elongation: " << cableElongation  << std::endl;
            //std::cout << "Max Tension : " << cableMaxTension  << std::endl;
            fprintf(outTetherLength, "%g \n", cableLength);
            fprintf(outTetherElongation, "%g \n", cableElongation);
            fprintf(outTetherMaxTension, "%g \n", cableMaxTension);

            // different segments of the cable
            VxData::Container* cableContainer = dynamic_cast<VxData::Container *>(cableExtension->getOutput(CableSystems::DynamicsICD::kCablesID));
            int numCables = (*cableContainer)[CableSystems::DynamicsICD::CablesContainerID::kNumCablesID].toInteger();
            //std::cout << "kNumCablesID: " << numCables  << std::endl;

            const VxData::FieldArrayBase& pointInfos =  dynamic_cast<const VxData::FieldArrayBase&>((*cableContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID]); 
            for (int cableIndex = 0; cableIndex<numCables; ++cableIndex)                    
            {
            // Get the Point Info container
            const VxData::Container& pointInfo = dynamic_cast<const VxData::Container &>(pointInfos.at(cableIndex));
            int numberOfPoints = pointInfo[CableSystems::DynamicsICD::PointInfoContainerID::kNumPointsID].toInteger();                        
            //std::cout << "Cable " << cableIndex << ": " << numberOfPoints << " points." << std::endl;

            // Get the Points positions 
            const VxData::Vector<Vx::VxVector3>& positionsCablePoints = dynamic_cast<const VxData::Vector<Vx::VxVector3> &>(pointInfo[CableSystems::DynamicsICD::PointInfoContainerID::kPositionsID]);
            const size_t positionsSize = positionsCablePoints.size();
            for ( size_t posIndex = 0; posIndex < positionsSize; posIndex++ )                        
            {
            const Vx::VxVector3& pos = positionsCablePoints[posIndex];                            
            //std::cout << "Position[" << posIndex << "]: " << pos << std::endl; 
            }
            }
            }
            }
            */

            // save positions and other data
            VxReal EkTot = 0.0;
            for (int j=0; j<NetSize; j++)
            {
                for (int i=0; i<NetSize; i++)
                {
                    //save for plots
                    VxVector3 pos = partMatrix[i][j]->getPosition();
                    VxVector3 vel = partMatrix[i][j]->getLinearVelocity();
                    //VxVector3 acc = partMatrix[i][j]->getLinearAcceleration();
                    VxReal Ek = partMatrix[i][j]->getKineticEnergy();
                    EkTot = EkTot + Ek;
                    //VxReal3 HVector;
                    //partMatrix[i][j]->getAngularMomentum(HVector);

                    //fprintf(outX, "%g ", pos.x() ); // out x to the file
                    //fprintf(outY, "%g ", pos.y() ); 
                    //fprintf(outZ, "%g ", pos.z() ); 
                    /*					fprintf(outVX, "%g ", vel.x() ); 
                    fprintf(outVY, "%g ", vel.y() ); 
                    fprintf(outVZ, "%g ", vel.z() ); 
                    fprintf(outKineticEnergy, "%g ", Ek ); 		*/			

                }
            }
            /*
            for (int j=0; j<4; j++)
            {
            VxVector3 pos = cornerMasses[j]->getPosition();
            VxVector3 vel = cornerMasses[j]->getLinearVelocity();
            VxVector3 acc = cornerMasses[j]->getLinearAcceleration();
            VxReal Ek = cornerMasses[j]->getKineticEnergy();
            EkTot = EkTot + Ek;
            VxReal3 HVector;
            cornerMasses[j]->getAngularMomentum(HVector);

            fprintf(outX, "%g ", pos.x() ); // out x to the file
            fprintf(outY, "%g ", pos.y() );
            fprintf(outZ, "%g ", pos.z() );
            fprintf(outVX, "%g ", vel.x() ); 
            fprintf(outVY, "%g ", vel.y() ); 
            fprintf(outVZ, "%g ", vel.z() ); 
            fprintf(outAX, "%g ", acc.x() );
            fprintf(outAY, "%g ", acc.y() ); 
            fprintf(outAZ, "%g ", acc.z() ); 
            fprintf(outKineticEnergy, "%g ", Ek ); 
            fprintf(outAngularMomentumX, "%g ", HVector[0]);
            fprintf(outAngularMomentumY, "%g ", HVector[1]);
            fprintf(outAngularMomentumZ, "%g ", HVector[2]);
            }
            */

            VxReal VeTot = 0.0; 
            int numConstraints;
            if (CMpresence == 1)
            {	numConstraints = 2*NetSize*(NetSize-1)+4; }
            else
            {	numConstraints = 2*NetSize*(NetSize-1); }

            for (int k=0; k<numConstraints; k++)
            {
                // save tension for attachDistance		
                VxReal TensionForce = allConstraintsVector[k]->getConstraintEquationForce(6);
                VxReal Ve = TensionForce * TensionForce / ( netYoungModulus * PI * pow(netRadius,2.00) / MeshLength );
                VeTot = VeTot + Ve;
                //fprintf(outF, "%1.15f ", TensionForce ); 

                //if ( (F!=0.0) && (checkF==0))
                //	checkF = 1;

            }

            currentTime = currentTime + timeStep;

            displayThreadsAndCornerMasses(gNetSize, gNetSize, indexCenterPart, winchRadius, chaserEdgeLength, targetEdgeLength, CMpresence, tetherPresence, blimpPresence);
        }
        application->endMainLoop();
        //VxRecorder::stop();

    }
    catch(const std::exception& ex )
    {
        //std::cout << "Error : " << ex.what() << std::endl;
        returnValue = 1;
    }
    catch( ... )
    {
        Vx::VxWarning(0, "Got an unhandled exception, application will exit!\n");
        returnValue = 1;
    }

    simulationTime = clock() - simulationTime;

    return returnValue;
}