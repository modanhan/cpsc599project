#pragma once

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
#include <set>

class MatchingMesh
{
	std::vector<bool> forceComputed;
	std::set<std::pair<unsigned int, unsigned int>> edges;
	std::set<int> affectedVertices;
	bool cDynamicFriction = 0;
	double staticFriction = 0.0025, dynamicFriction = 0.001;
public:
	double radius = 0.01;
	chai3d::cMesh* mesh;
	std::vector<chai3d::cVector3d> originalPoints = std::vector<chai3d::cVector3d>(3);
	std::vector<chai3d::cVector3d> forces = std::vector<chai3d::cVector3d>(3);
	std::vector<chai3d::cVector3d> velocities = std::vector<chai3d::cVector3d>(3);

	double m = 1, k = 100, b = 10;

	bool contact = 0;
	chai3d::cVector3d lCursorPosition;

	MatchingMesh(chai3d::cWorld* world);

	void update(chai3d::cVector3d &force, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition);
	bool updateTriangle(chai3d::cVector3d &force, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition, unsigned int);
	bool updateLine(unsigned int indexP0, unsigned int indexP1, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition);
	bool updatePoint(unsigned int index, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition);
	void movePoint(unsigned int index);

	MatchingMesh();
	~MatchingMesh();
};

