// Created by Eleonora Botta, September 30, 2016
// Modified by Eleonora Botta, November 3, 2016: save on a separate file the positions of the cable points (for all cables)
// Modified by Eleonora Botta, November 5, 2016: rearranged the file with the positions of the cable points (for all cables)
// Modified by Eleonora Botta, December 28, 2016: added a version of function without outTetherPoints

// Save dynamics of the tether and winch linear velocity

#include "MyLibraryNet/TestFunction.h"
#include "MyLibraryNet/OrbitalMech2BP_Part.h"

#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>
#include <VxDynamics/Hinge.h>
#include <Vx/VxHinge.h>
#include <VxMath/Transformation.h>
#include <Vx/VxEulerAngles.h>
#include <Vx/VxMessage.h>
#include <iostream>
#include <VxContent/Scene.h>
#include <VxContent/SceneICD.h>
#include <VxContent/ConnectionContainerExtension.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/MechanismICD.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Prismatic.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Sphere.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxDynamics/ConstraintEquationRelaxationContainer.h>
#include <VxDynamics/Hinge.h>
#include <Vx/VxCylindrical.h>
#include <Vx/VxGearRatio.h>
#include <Vx/VxHinge.h>
#include <Vx/VxHomokinetic.h>
#include <Vx/VxPrismatic.h>
#include <Vx/VxScrewJoint.h>
#include <Vx/VxSpring.h>
#include <Vx/VxDistanceJoint.h>
#include <Vx/VxAngular2Position3.h>
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxSolverParameters.h>
#include <Vx/VxMaterialTable.h>
#include <VxMath/BoundingSphere.h>
#include <VxSim/IMobile.h>
#include <Vx/VxAssembly.h>
#include <Vx/VxBox.h>
#include <Vx/VxSphere.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxPart.h>
#include <Vx/VxTransform.h>
#include <VxSim/VxApplication.h>
#include <VxDynamics/Mechanism.h> 	// this is needed by VxSmartInterface<IExtension> cableExtension

#include <VxData/FieldArray.h>
#include <VxData/Vector.h>
#include <Vx/VxSmartPtr.h>

#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;
using namespace CableSystems::DynamicsICD::PointInfoContainerID;



void TestFunction(VxSmartInterface<IExtension> cableExtension, double mu)
{
	VxSim::VxSmartInterface<IDynamics> dynamicsInterface(cableExtension);
			//Vx::VxInputOutputContainer& G = cableExtension->getProxy()->getFieldsContainer();
			VxDynamicsObject* debugGroup = dynamicsInterface->getDebugGroup();

			auto Assem = debugGroup->getRegisteredAssemblies();
			auto Off = Assem.size();
			auto assemblyiter = Assem.begin() + 1;
			auto assemblyiter1 = Assem.begin();
			VxAssembly* someassembly = *assemblyiter;
			VxAssembly* someassembly1 = *assemblyiter1;
			VxCylindrical* tethercylindrical;
			const VxConstraintSet& constraintsinassembly = someassembly->getConstraints(); //-> this should have the cable in it, but it doesnt
			const VxConstraintSet& constraintsinassembly1 = someassembly1->getConstraints();
			int H = 1;

	//std::cout << "Size of constraint set in assembly is " << constraintsinassembly.size() << std::endl;

	for (auto iter = constraintsinassembly1.begin(), end = constraintsinassembly1.end(); iter != end; ++iter)

	{

		VxConstraint* cons = *iter;

		if (iter == constraintsinassembly1.begin()) {

			VxCylindrical* cyl = dynamic_cast<VxCylindrical*>(cons);



			if (cyl != nullptr)

			{



				// ...cons is a cylindrical...

				tethercylindrical = cyl;
				Vx::VxPart* Part = tethercylindrical->getPart(1);
				
				Part->addForce(OrbitalMech2BP_Part(Part,mu));


			}

		}

	}

}