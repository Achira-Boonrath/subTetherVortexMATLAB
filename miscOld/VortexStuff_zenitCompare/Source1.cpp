// read constants
std::stringstream constantsFileName;
constantsFileName << pathData << "constants.txt";
VxReal PI = readParameter<VxReal>("PI", constantsFileName);

// read data for simulation
std::stringstream simulationDataFileName;
simulationDataFileName << pathData << "simulationdata.txt";
int netModel = readParameter<int>("netModel", simulationDataFileName);
int CMsPresent = readParameter<int>("CMsPresent", simulationDataFileName);

// read data for net
std::stringstream netDataFileName;
netDataFileName << pathData << "netData.txt";
int NetSize = readParameter<int>("NetSize", netDataFileName);
int NI = 0;

if (netModel == 5) //CW, 7/1/21 - only read when net model is 5
{
	NI = readParameter<int>("NI", netDataFileName);
}

VxReal NetSideLength = readParameter<VxReal>("NetSideLength", netDataFileName);
VxReal lengthKnotThread = readParameter<VxReal>("lengthKnotThread", netDataFileName);
VxReal Z = readParameter<VxReal>("Z", netDataFileName);
VxReal compression = readParameter<VxReal>("compression", netDataFileName);
VxReal netRadius = readParameter<VxReal>("netRadius", netDataFileName);
VxReal netYoungModulus = readParameter<VxReal>("netYoungModulus", netDataFileName);
VxReal netDensity = readParameter<VxReal>("netDensity", netDataFileName);
VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
VxReal csiBending = readParameter<VxReal>("csiBending", netDataFileName);
VxReal netEjectionVelocity = readParameter<VxReal>("netEjectionVelocity", netDataFileName);
int wayPointsNb, bendingPresentInNet, netCGtype;
bendingPresentInNet = readParameter<int>("bendingPresentInNet", netDataFileName);

if (netModel == 1)
{
	wayPointsNb = readParameter<int>("wayPointsNb", netDataFileName);
	netCGtype = readParameter<int>("netCGtype", netDataFileName);
}

// define some useful quantities
VxReal MeshLength = NetSideLength / (NetSize - 1);

//VxReal NodeLength = MeshLength / (NI + 1);
//VxReal stepX = (MeshLength * compression) / (NI + 1);
//VxReal stepY = (MeshLength * compression) / (NI + 1);

////////???????????????
int ucount = NetSize + (NetSize - 1) * NI; // Number of nodes on a side
int vcount = NetSize + (NetSize - 1) * NI;
assert(ucount < MAXGRIDSIZE&& vcount < MAXGRIDSIZE);

///////////////////////////////

// read data for corner thread if CMs present (needed to compute mass of net corner masses)
VxReal cornerThreadLength;
VxReal cornerThreadRadius;
if (CMsPresent == 1)
{
	std::stringstream CMsDataFileName;
	CMsDataFileName << pathData << "CMsData.txt";
	cornerThreadLength = readParameter<VxReal>("cornerThreadLength", CMsDataFileName);
	cornerThreadRadius = readParameter<VxReal>("cornerThreadRadius", CMsDataFileName);
	if (cornerThreadLength == -1)
		cornerThreadLength = MeshLength * sqrt(2.0);
}
VxReal N_CornerThread = ceil(cornerThreadLength / NodeLength);
VxReal cornerNodeLength = cornerThreadLength / N_CornerThread;
VxReal massHalfCornerThread = netDensity * cornerThreadLength * PI * pow(cornerThreadRadius, 2) / 2;
VxReal massInnerCornerThread = netDensity * cornerNodeLength * PI * pow(cornerThreadRadius, 2) / 2;

// find threads' stiffness
VxReal threadStiffness = netYoungModulus * PI * pow(netRadius, 2.00) / NodeLength;
VxReal threadBendingStiffness = netYoungModulus * PI / 4.0 * pow(netRadius, 4) / NodeLength; // EI/l
VxReal threadTransverseStiffness = 3.0 * netYoungModulus * PI / 4.0 * pow(netRadius, 4) / pow(MeshLength, 3); // 3EI/l^3

// find natural frequency for bending, found for thread 1 simply, and write it in a file 
// for axial, call function that computes it from mass and stiffness matrices for the net
// need the mass of  a sample knot:
VxReal massHalfThread = netDensity * MeshLength * PI * pow(netRadius, 2) / 2;
VxReal massKnot = netDensity * lengthKnotThread * PI * pow(netRadius, 2);
VxReal partMass = 4 * massHalfThread + massKnot;		// most of the net nodes' mass
VxReal partInertia = partMass * pow(MeshLength, 2);		// most of the net nodes' moment of inertia, cantilever model
double wn1Axial = findNetFirstAxialNatFreq(NetSize, CMsPresent, massHalfThread, massHalfCornerThread, massKnot, threadStiffness);
float wn1BendingRot = pow(threadBendingStiffness / partInertia, 0.5);
float wn1BendingTransl = pow(threadTransverseStiffness / partMass, 0.5);
writeNaturalFrequencies(pathData, wn1Axial, wn1BendingRot, wn1BendingTransl);

// find threads' damping
VxReal threadDamping = 2 * csiAxial / wn1Axial * threadStiffness; // TO BE CHANGED
VxReal threadBendingDamping = 2 * csiBending / wn1BendingRot * threadBendingStiffness;  // da cambiare
VxReal threadTransverseDamping = 2 * csiBending / wn1BendingTransl * threadTransverseStiffness; // da cambiare

//CW - 6/1/2022 - Create vectors of objects (parts and distance joints) + matrices that contain information about the objects
VxMaterial* thispartmaterial;
VxVector3 lowerleftcorner(0, 0, Z); // position of part [0][0]

int knotCount = pow(NetSize,2);
int threadCount = 2 * (NetSize*(NetSize - 1));

int q = 0; 
int numNodes = 0;
for (int i = 0; i < NetSize - 1; i++)
{
	for (int j = 0; j < NetSize - 1; j++)
	{

		nodeMatrix[q][0] = i; //store i and j index for each knot in nodeMatrix
		nodeMatrix[q][1] = j;

		//determine which threads each node touches (receives mass contribution from) based on i and j
		// '0' refers to no thread at all, since we can not have empty elements in arrays of integers
		if i == 0 && j == 0; //node at origin corner
		{
			nodeMatrix[q][2] = 0; //thread below
			nodeMatrix[q][3] = 0; //thread to the left
			nodeMatrix[q][4] = 1; //thread to the right
			nodeMatrix[q][5] = NetSize; //thread above
		}

		else if i == NetSize - 1 && j == 0 //corner
		{
			nodeMatrix[q][2] = 0; //thread below
			nodeMatrix[q][3] = (NetSize - 1); //thread to the left
			nodeMatrix[q][4] = 0; //etc. for all thread assignments
			nodeMatrix[q][5] = (NetSize - 1) + (1 * NetSize);
		}

		else if i == 0 && j == NetSize - 1 //corner
		{
			nodeMatrix[q][2] = (j)*(NetSize - 1) + (j - 1)*(NetSize)+1;
			nodeMatrix[q][3] = 0;
			nodeMatrix[q][4] = (j)*(NetSize - 1) + (j)*(NetSize)+1;
			nodeMatrix[q][5] = 0;
		}

		else if i == NetSize - 1 && j == NetSize - 1; //node at farthest corner from origin
		{
			nodeMatrix[q][2] = (j)*(NetSize - 1) + (j)*(NetSize);
			nodeMatrix[q][3] = (j + 1)*(NetSize - 1) + (j)*(NetSize);  
			nodeMatrix[q][4] = 0;
			nodeMatrix[q][5] = 0;
		}

		else if (i != 0 && j == 0) //nodes on bottom edge
		{
			nodeMatrix[q][2] = 0;
			nodeMatrix[q][3] = i;
			nodeMatrix[q][4] = i+1;
			nodeMatrix[q][5] = i + NetSize;
		}

		else if (i != 0 && j == NetSize - 1)//nodes on top edge
		{
			nodeMatrix[q][2] = i - NetSize + j((2 * NetSize) - 1) + 1;
			nodeMatrix[q][3] = i + j((2 * NetSize) - 1);
			nodeMatrix[q][4] = i + j((2 * NetSize) - 1) + 1;
			nodeMatrix[q][5] = 0;
		}

		else if (i == 0 && j != 0) //nodes on left edge
		{
			nodeMatrix[q][2] = j*(NetSize - 1) + (j - 1)*NetSize + 1;
			nodeMatrix[q][3] = 0;
			nodeMatrix[q][4] = j*(NetSize)+j*(NetSize - 1) + 1; 
			nodeMatrix[q][5] = (NetSize - 1) + j*((2 * NetSize) - 1) + 1;
	
		}

		else if (i == NetSize - 1 && j != 0) //nodes on right edge
		{
			nodeMatrix[q][2] = (j + 1)*(NetSize - 1) + (j - 1)*NetSize +  1;
			nodeMatrix[q][3] = j*(NetSize)+j*(NetSize - 1) + (NetSize - 2) + 1;
			nodeMatrix[q][4] = 0;
			nodeMatrix[q][5] = 2 * (NetSize - 1) + j*((2 * NetSize) - 1) + 1;
		}

		else //nodes that do not lie on an edge
		{
			nodeMatrix[q][2] = (NetSize - 1)*j + (j - 1)*(NetSize)+i + 1;
			nodeMatrix[q][3] = (NetSize - 1)*j + (j - 1)*(NetSize)+i + NetSize;
			nodeMatrix[q][4] = (NetSize - 1)*j + (j - 1)*(NetSize)+i + NetSize + 1;
			nodeMatrix[q][5] = (NetSize - 1)*j + (j - 1)*(NetSize)+i + 2*NetSize;
		}

		q = q + 1;
		numNodes = q;
	}
}

//create parts based on information from nodeMatrix
for (int q = 0; q <= numNodes; i++)
{

	VxSmartInterface<Part> p = VxExtensionFactory::create(PartICD::kFactoryKey);
	partMass = 0;
	for (int j = 2; j <= 5; j++)
	{
		int thread = nodeMatrix[q][j];
		if thread != 0
		{
			int nodesOnThread = nodesOnThread(nodesMatrix[], thread, numNodes);
			partMass = partMass + (threadMass / nodesOnThread); 
		}
	}
	p->parameterMassPropertiesContainer.mass = partMass;

	// part material, depending on part location
	thispartmaterial = myNetMaterial11;
	if (((i == 0) && (j == 0)) || ((i == NetSize - 1) && (j == 0)) || ((i == 0) && (j == NetSize - 1)) || ((i == NetSize - 1) && (j == NetSize - 1)))   // corner part 
	{
		thispartmaterial = myNetMaterial00;
	}
	else if (((i > 0) && (i < NetSize - 1) && (j == 0)) || ((i > 0) && (i < NetSize - 1) && (j == NetSize - 1)) || ((j > 0) && (j < NetSize - 1) && (i == 0)) || ((j > 0) && (j < NetSize - 1) && (i == NetSize - 1)))   // side part 
	{
		thispartmaterial = myNetMaterial01;
	}

	VxSmartInterface<Sphere> sphere = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey);
	VxReal sphereRadius = pow((0.75 / PI * partMass / netDensity), (1 / 3.)); // x 10 for visualization only;
	sphere->parameterRadius = sphereRadius;
	VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey);
	cg = sphere;
	cg->parameterMaterial = thispartmaterial;
	p->addCollisionGeometry(cg);
	p->autoComputeInertiaAndCOM();
	
	VxVector3 pos = lowerleftcorner + VxVector3(stepX * nodesMatrix[q][1], stepY * nodesMatrix[q][2], 0);
	p->setLocalTransform(VxMath::Transformation::createTranslation(pos));
	p->setLinearVelocity(Vx::VxVector3(0.0, 0.0, -netEjectionVelocity));
	partMatrix[q] = p;

	assembly->addPart(p);
}

//create distance joints
int counter = 0;
for (int q = 0; q <= numNodes; j++)
{

	int i = nodeMatrix[q][0];
	int j = nodeMatrix[q][1];

	for (int r = 0; r <= numNodes; i++)
	{

		i_next = nodeMatrix[r][0];
		j_next = nodeMatrix[r][1];

		//use knowledge about orientation of threads relative to nodes, + the number of nodes on threads, to create distance joints
		istep = 1 / nodesOnThread(nodeMatrix, nodeMatrix[q][4]);
		jstep = 1 / nodesOnThread(nodeMatrix, nodeMatrix[q][5]);

		if (i_next == i + istep && j_next == j) || (j_next == j + jstep && i_next = i)
		{
			if (netModel == 0) // no model of bending stiffness
			{
				if (j_next == j) //stepping across i
				{
					threadLength = MeshLength*i_step;
				}
				else //stepping across j 
				{ 
					threadLength = MeshLength*j_step;
				}

				if (bendingPresentInNet == 0)
				{
					allConstraintsVector[counter] = attachDistance(assembly, partMatrix[q], partMatrix[r], threadLength, threadStiffness, threadDamping); //TO CHANGE: shouldn't need meshlength
					distanceJointMatrix[counter][0] = i; //store distance joint attachment points - row number of distanceJointMatrix matches to location in allConstraintsVector
					distanceJointMatrix[counter][1] = j;
					distanceJointMatrix[counter][2] = i_next;
					distanceJointMatrix[counter][3] = j_next;
					break
				}
				else if (bendingPresentInNet == 1)
				{
					allConstraintsVector[counter] = attachDistance(assembly, partMatrix[q], partMatrix[k], threadLength, threadStiffness, threadDamping);
					allConstraintsVectorPrismatic[counter] = attachPrismatic(assembly, partMatrix[q], partMatrix[r], threadLength, threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, threadTransverseStiffness, threadTransverseDamping);
					distanceJointMatrix[counter][0] = i;
					distanceJointMatrix[counter][1] = j;
					distanceJointMatrix[counter][2] = i_next;
					distanceJointMatrix[counter][3] = j_next;
					break
				}
			}
			else if (netModel == 1)
			{
				addCableForNetThread(mechanism, partMatrix[q], partMatrix[r], threadStiffness, threadDamping, threadBendingStiffness, threadBendingDamping, threadLength, netRadius, linearNetDensity, wayPointsNb, bendingPresentInNet, netCGtype);
				distanceJointMatrix[counter][0] = i;
				distanceJointMatrix[counter][1] = j;
				distanceJointMatrix[counter][2] = i_next;
				distanceJointMatrix[counter][3] = j_next;
				break
			}
		
		}
	}
	
		
}


int nodesOnThread(int nodesMatrix[][], int thread) {
	
	//counts the number of nodes that touch (receive mass contribution from) a given thread 

	int count = 0;

	else
	{
		for (int i = 0; i <= sizeof(nodesMatrix) / sizeof(nodesMatrix[0]); i++)
		{
			for (int j = 2; j <= 5; j++)
			{
				if (arr[i][j] = thread)
				{
					count++;
				}

				else
				{
					break;
				}
			}
		}
	}

	return count;
}
	



