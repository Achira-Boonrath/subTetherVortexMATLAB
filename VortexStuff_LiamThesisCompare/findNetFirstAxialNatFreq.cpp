// Created by Eleonora Botta, September 10, 2016
// Create a net made out of sphere nodes and cables between them simulated by distance or other joints in the xy plane, at z=Z, and centered at x,y=0. 
// Modified by Eleonora Botta, March 10, 2017: consider mass of knots in finding 1st natural frequency

#include "MyLibraryNet/findNetFirstAxialNatFreq.h"

#include <iostream>
#include <fstream>
#include <sstream>

// third party libraries
#include <Eigen/Eigen/Core> // for computing eigenvalues
#include <Eigen/Eigen/Dense>
#include <Eigen/Eigen/Eigenvalues>
using namespace Eigen;

double findNetFirstAxialNatFreq(int NetSize, int CMsPresent, float massHalfThread, float massHalfCornerThread, float massKnot, float threadStiffness )
{
	std::cout << "I am computing the first nat. freq. " << std::endl;

	// create mass matrix
	Eigen::MatrixXf massMatrix = Eigen::MatrixXf::Zero(3*NetSize*NetSize, 3*NetSize*NetSize);	
	int totalIndex = 0;
	for (int i=0; i<NetSize; i++)
	{
        for (int j=0; j<NetSize; j++)
		{
			// find mass of this node
			float partMass = 4 * massHalfThread + massKnot; // central part 
				if ( ( (i==0)&&(j==0) ) || ( (i==NetSize-1)&&(j==0) ) || ( (i==0)&&(j==NetSize-1) ) || ( (i==NetSize-1)&&(j==NetSize-1) )  )   // corner part 
				{
					partMass = 2 * massHalfThread + massKnot; 
					if (CMsPresent==1)
						partMass = partMass + massHalfCornerThread;
				}
				else if ( ( (i>0) && (i<NetSize-1) && (j==0) ) || ( (i>0) && (i<NetSize-1) && (j==NetSize-1) ) || ( (j>0) && (j<NetSize-1) && (i==0) ) || ( (j>0) && (j<NetSize-1) && (i==NetSize-1) )  )   // side part 
				{
					partMass = 3 * massHalfThread + massKnot;
				}

			
			// put this value in the mass matrix
			totalIndex = 1 + i + NetSize*j;
			//std::cout << i << "," << j << "," << totalIndex << '\n';
			massMatrix( 3*(totalIndex-1) , 3*(totalIndex-1) ) = partMass;
			massMatrix( 3*(totalIndex-1)+1 , 3*(totalIndex-1)+1 ) = partMass;
			massMatrix( 3*(totalIndex-1)+2 , 3*(totalIndex-1)+2 ) =  partMass;
		}
	}

	// create stiffness matrix
	Eigen::MatrixXf stiffnessMatrix = Eigen::MatrixXf::Zero(3*NetSize*NetSize, 3*NetSize*NetSize);	
	int totalIndexNode1;
	int totalIndexNode2;
	for (int j=0; j<=NetSize-1; j++)
    {
        for (int i=0; i<=NetSize-1; i++)
		{
			if (i < NetSize-1)
			{
				// horizontal -> corresponding to x rows and columns:
				// there is a thread between node1(i,j) and node2(i+1,j)  
				// give positive stiffness to element in corresponding row, column of matrix: 
				totalIndexNode1 = 1 + i + NetSize*j;
				totalIndexNode2 = 1 + (i+1) + NetSize*j;
				stiffnessMatrix( 3*(totalIndexNode1-1) , 3*(totalIndexNode1-1) ) = stiffnessMatrix( 3*(totalIndexNode1-1) , 3*(totalIndexNode1-1) ) + threadStiffness; 
				stiffnessMatrix( 3*(totalIndexNode2-1) , 3*(totalIndexNode2-1) ) = stiffnessMatrix( 3*(totalIndexNode2-1) , 3*(totalIndexNode2-1) ) + threadStiffness; 
				// give negative stiffness to element in mixed row, column of matrix:
				stiffnessMatrix( 3*(totalIndexNode1-1) , 3*(totalIndexNode2-1) ) = stiffnessMatrix( 3*(totalIndexNode1-1) , 3*(totalIndexNode2-1) ) - threadStiffness; 
				stiffnessMatrix( 3*(totalIndexNode2-1) , 3*(totalIndexNode1-1) ) = stiffnessMatrix( 3*(totalIndexNode2-1) , 3*(totalIndexNode1-1) ) - threadStiffness; 
			}
			//std::cout << i << "," << j << "," << totalIndexNode1 << "," << totalIndexNode2 << '\n';
			if (j < NetSize-1)
			{
				// vertical -> corresponding to y rows and columns:
				// there is a thread between node1(i,j) and node2(i,j+1)  
				// give positive stiffness to element in corresponding row, column of matrix: 
				totalIndexNode1 = 1 + i + NetSize*j;
				totalIndexNode2 = 1 + i + NetSize*(j+1);
				stiffnessMatrix( 3*(totalIndexNode1-1)+1 , 3*(totalIndexNode1-1)+1 ) = stiffnessMatrix( 3*(totalIndexNode1-1)+1 , 3*(totalIndexNode1-1)+1 ) + threadStiffness; 
				stiffnessMatrix( 3*(totalIndexNode2-1)+1 , 3*(totalIndexNode2-1)+1 ) = stiffnessMatrix( 3*(totalIndexNode2-1)+1 , 3*(totalIndexNode2-1)+1 ) + threadStiffness; 
				// give negative stiffness to element in mixed row, column of matrix:
				stiffnessMatrix( 3*(totalIndexNode1-1)+1 , 3*(totalIndexNode2-1)+1 ) = stiffnessMatrix( 3*(totalIndexNode1-1)+1 , 3*(totalIndexNode2-1)+1 ) - threadStiffness; 
				stiffnessMatrix( 3*(totalIndexNode2-1)+1 , 3*(totalIndexNode1-1)+1 ) = stiffnessMatrix( 3*(totalIndexNode2-1)+1 , 3*(totalIndexNode1-1)+1 ) - threadStiffness; 
			}
			//std::cout << i << "," << j << "," << totalIndexNode1 << "," << totalIndexNode2 << '\n';
		}
	}

	// find matrix A = (M^-1)K
	Eigen::MatrixXf matrixA = ( massMatrix.inverse() )*stiffnessMatrix;

	// find eigenvalues
	Eigen::EigenSolver<Eigen::MatrixXf> es;
	es.compute(matrixA, /* computeEigenvectors = */ false);

	//std::cout << "The eigenvalues of A are: " << es.eigenvalues() << std::endl;
	//std::cout << "Number of eigenvalues: " << es.eigenvalues().size() << std::endl;
	//std::cout << es.eigenvalues()[es.eigenvalues().size()-2]<< std::endl;
	//std::cout << es.eigenvalues()[es.eigenvalues().size()-2].real() << std::endl;

	// find first natural frequency
	double wn1Axial = 10000.0;
	double currentEigenvalue;
	for (int i=0; i<=es.eigenvalues().size(); i++)
	{
		currentEigenvalue = es.eigenvalues()[i].real();
		if ( ( pow(currentEigenvalue,0.5) < wn1Axial) && (  pow(currentEigenvalue,0.5) > 10) )
			wn1Axial = pow(currentEigenvalue,0.5);
	}
	//std::cout << "Min eigenvalue is: " << wn1Axial << std::endl;

	/*
	// write
	std::stringstream massFileName;
	massFileName << "mass.txt" ;
	std::ofstream outFile;

	outFile.open(massFileName.str());
	std::string a; 
	if (outFile.is_open()) 
	{
		outFile << massMatrix << '\n'; 
	}
	outFile.close();

	// write
	std::stringstream stiffnessFileName;
	stiffnessFileName << "stiffness.txt" ;

	outFile.open(stiffnessFileName.str());
	if (outFile.is_open()) 
	{
		outFile << stiffnessMatrix << '\n'; 
	}
	outFile.close();

	// write
	std::stringstream aFileName;
	aFileName << "A.txt" ;

	outFile.open(aFileName.str());
	if (outFile.is_open()) 
	{
		outFile << ( massMatrix.inverse() )*stiffnessMatrix << '\n'; 
	}
	outFile.close();


	// write
	std::stringstream eigFileName;
	eigFileName << "eigenvalues.txt" ;

	outFile.open(eigFileName.str());
	if (outFile.is_open()) 
	{
		outFile << es.eigenvalues() << '\n'; 
	}
	outFile.close();
	*/

	std::cout << "I found the first nat. freq. " << std::endl;
	return wn1Axial;
}