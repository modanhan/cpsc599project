#pragma once

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------

class MatchingMesh
{
public:
	double radius = 0.01;
	chai3d::cMesh * object;
	chai3d::cMesh* mesh;
	std::vector<chai3d::cVector3d> originalPoints = std::vector<chai3d::cVector3d>(3);
	std::vector<chai3d::cVector3d> forces = std::vector<chai3d::cVector3d>(3);
	std::vector<chai3d::cVector3d> velocities = std::vector<chai3d::cVector3d>(3);

	double m = 1, k = 100, b = 10;

	MatchingMesh(chai3d::cWorld* world);

	void update(chai3d::cVector3d &force, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition);
	bool updateTriangle(chai3d::cVector3d &force, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition, unsigned int);
	bool updateLine(unsigned int indexP0, unsigned int indexP1, chai3d::cVector3d &position);
	bool updatePoint(unsigned int index, chai3d::cVector3d &position);
	void movePoint(unsigned int index);

	MatchingMesh();
	~MatchingMesh();
};

