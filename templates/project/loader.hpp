#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "chai3d.h"

using namespace chai3d;
using namespace std;

void loadOBJ(string, vector<cVector3d> *vertices, vector<cVector3d> *normals, vector<cVector3d> *uvs, vector<unsigned int> *indices);