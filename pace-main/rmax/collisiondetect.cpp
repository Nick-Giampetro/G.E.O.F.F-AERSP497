//#include <iostream>
#include <math.h>
#include "collisiondetect.h"

//using namespace std;

/* This function calculates the intersection of a line and a triangle 
   by Moller's algorithm (one of the most fundamental methods) */
double CollisionDetectLineVSTri(double position[3], double line[3], double vertex0[3], double vertex1[3], double vertex2[3])
{

/*
	position	: position vector of the UAV
	line		: line vector of the Laser
	triangle	: verteces of the 3 points of surface of obstacle
*/

	double a0[3],a1[3],a2[3];
	double b[3];

	double det;
	double LengthVariable,u,v;

	a0[0] = -line[0];
	a0[1] = -line[1];
	a0[2] = -line[2];

	a1[0] = vertex1[0] - vertex0[0];
	a1[1] = vertex1[1] - vertex0[1];
	a1[2] = vertex1[2] - vertex0[2];


	a2[0] = vertex2[0] - vertex0[0];
	a2[1] = vertex2[1] - vertex0[1];
	a2[2] = vertex2[2] - vertex0[2];

	b[0] = position[0] - vertex0[0];
	b[1] = position[1] - vertex0[1];
	b[2] = position[2] - vertex0[2];


	det = (a0[0]*a1[1]*a2[2] + a1[0]*a2[1]*a0[2] + a0[1]*a1[2]*a2[0])
			- (a2[0]*a1[1]*a0[2] + a1[0]*a0[1]*a2[2] + a2[1]*a1[2]*a0[0]);

	if(fabs(det) < 0.001)
	{
		//cout << "Both are parallel" << endl;
		LengthVariable = 3.0; // these 3 numbers are meaningless.
		u = -3.0;
		v = -3.0;
	}
	else
	{
		LengthVariable = ((b[0]*a1[1]*a2[2] + a1[0]*a2[1]*b[2] + b[1]*a1[2]*a2[0])
			- (a2[0]*a1[1]*b[2] + a1[0]*b[1]*a2[2] + a2[1]*a1[2]*b[0]))/det;

		u = ((a0[0]*b[1]*a2[2] + b[0]*a2[1]*a0[2] + a0[1]*b[2]*a2[0])
			- (a2[0]*b[1]*a0[2] + b[0]*a0[1]*a2[2] + a2[1]*b[2]*a0[0]))/det;

		v = ((a0[0]*a1[1]*b[2] + a1[0]*b[1]*a0[2] + a0[1]*a1[2]*b[0])
			- (b[0]*a1[1]*a0[2] + a1[0]*a0[1]*b[2] + b[1]*a1[2]*a0[0]))/det;
	}

/*	cout << "det = " << det << endl;
	cout << "LengthVariable = " << LengthVariable << endl;
	cout << "u = " << u << endl;
	cout << "v = " << v << endl;  */


	if((LengthVariable < 0.0) || (LengthVariable > 1.0))
	{
		LengthVariable = 2.0;
		//cout << "NO OBSTACLE by t" << endl;
	}
	else if((u < 0.0) || (v < 0.0))
	{
		LengthVariable = 2.0;
		//cout << "NO OBSTACLE by u , v" << endl;
	}
	else if(u + v > 1.0)
	{
		LengthVariable = 2.0;
		//cout << "NO OBSTACLE by u + v" << endl;
	}
	else
	{
		//cout << "OBSTACLE DETECTED!!!!!" << endl;
	}

	return LengthVariable;
}

// this function calculate the intersection of line and parallelogram
// The input vertex "vertex0" must be between "1" and "2".
double CollisionDetectLineVSPara(double position[3], double line[3], double vertex0[3], double vertex1[3], double vertex2[3])
{

/*
	position	: position vector of the UAV
	line		: line vector of the laser
	triangle	: vertices of the 3 points of obstacle surface
*/

	double a0[3],a1[3],a2[3];
	double b[3];

	double det;
	double LengthVariable,u,v;

	a0[0] = -line[0];
	a0[1] = -line[1];
	a0[2] = -line[2];

	a1[0] = vertex1[0] - vertex0[0];
	a1[1] = vertex1[1] - vertex0[1];
	a1[2] = vertex1[2] - vertex0[2];

	a2[0] = vertex2[0] - vertex0[0];
	a2[1] = vertex2[1] - vertex0[1];
	a2[2] = vertex2[2] - vertex0[2];

	b[0] = position[0] - vertex0[0];
	b[1] = position[1] - vertex0[1];
	b[2] = position[2] - vertex0[2];


	det = (a0[0]*a1[1]*a2[2] + a1[0]*a2[1]*a0[2] + a0[1]*a1[2]*a2[0])
			- (a2[0]*a1[1]*a0[2] + a1[0]*a0[1]*a2[2] + a2[1]*a1[2]*a0[0]);

	if(fabs(det) < 0.001)
	{
		//cout << "Both are parallel" << endl;
		LengthVariable = 3.0;// these 3 numbers are meaningless.
		u = -3.0;
		v = -3.0;
	}
	else
	{
		LengthVariable = ((b[0]*a1[1]*a2[2] + a1[0]*a2[1]*b[2] + b[1]*a1[2]*a2[0])
			- (a2[0]*a1[1]*b[2] + a1[0]*b[1]*a2[2] + a2[1]*a1[2]*b[0]))/det;

		u = ((a0[0]*b[1]*a2[2] + b[0]*a2[1]*a0[2] + a0[1]*b[2]*a2[0])
			- (a2[0]*b[1]*a0[2] + b[0]*a0[1]*a2[2] + a2[1]*b[2]*a0[0]))/det;

		v = ((a0[0]*a1[1]*b[2] + a1[0]*b[1]*a0[2] + a0[1]*a1[2]*b[0])
			- (b[0]*a1[1]*a0[2] + a1[0]*a0[1]*b[2] + b[1]*a1[2]*a0[0]))/det;
	}

	
/*	cout << "det = " << det << endl;
	cout << "LengthVariable = " << LengthVariable << endl;
	cout << "u = " << u << endl;
	cout << "v = " << v << endl;  */


	if((LengthVariable < 0.0) || (LengthVariable > 1.0))
	{
		LengthVariable = 2.0;
		//cout << "NO OBSTACLE by t" << endl;
	}
	else if((u < 0.0) || (u > 1.0))
	{
		LengthVariable = 2.0;
		//cout << "NO OBSTACLE by u" << endl;
	}
	else if((v < 0.0) || (v > 1.0))
	{
		LengthVariable = 2.0;
		//cout << "NO OBSTACLE by v" << endl;
	}
	else
	{
		//cout << "OBSTACLE DETECTED!!!!!" << endl;
	}

	return LengthVariable;
}


// this function is to multiply 3x3 mat by a 3-component vector.
void multiply3x3byVec(double mat[3][3], double vector[3])
{

	double a[3];

	a[0] = mat[0][0]*vector[0] + mat[0][1]*vector[1] + mat[0][2]*vector[2];
	a[1] = mat[1][0]*vector[0] + mat[1][1]*vector[1] + mat[1][2]*vector[2];
	a[2] = mat[2][0]*vector[0] + mat[2][1]*vector[1] + mat[2][2]*vector[2];

	vector[0] = a[0];
	vector[1] = a[1];
	vector[2] = a[2];

}

// this function is to add 3-component vectors with each other.
void addVecAndVec( double added[3], double original[3])
{

	original[0]	+= added[0];
	original[1] += added[1];
	original[2] += added[2];

}

