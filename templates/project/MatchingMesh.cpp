#include "MatchingMesh.h"
#include "loader.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace chai3d;
using namespace Eigen;

cVector3d prevPos = cVector3d(0,0,0);

MatchingMesh::MatchingMesh(cWorld* world, string s)
{
	mesh = new cMesh();

	vector<cVector3d> vertices;
	vector<cVector3d> normals;
	vector<cVector3d> uvs;
	vector<unsigned int> indices;

	loadOBJ(s, &vertices, &normals, &uvs, &indices);

	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		mesh->newVertex(vertices[i], normals[i]);
	}
	mesh->setVertexColor(cColorf(1, 1, 1, 1));

	for (unsigned int i = 0; i < indices.size() / 3; i++)
	{
		mesh->newTriangle(indices[i * 3], indices[i * 3 + 1], indices[i * 3 + 2]);
		vector<int> sorted_idx; sorted_idx.push_back(indices[i * 3 + 0]); sorted_idx.push_back(indices[i * 3 + 1]); sorted_idx.push_back(indices[i * 3 + 2]);
		sort(sorted_idx.begin(), sorted_idx.end());
		// ensures no duplicated/directional edges by making sure edge.first < edge.second
		edges.insert(pair<unsigned int, unsigned int>(sorted_idx[0], sorted_idx[1]));
		edges.insert(pair<unsigned int, unsigned int>(sorted_idx[0], sorted_idx[2]));
		edges.insert(pair<unsigned int, unsigned int>(sorted_idx[1], sorted_idx[2]));
	}
	mesh->computeAllNormals();

	mesh->m_material = cMaterial::create();
	mesh->m_material->setWhiteGhost();
	mesh->m_material->setUseHapticShading(true);

	world->addChild(mesh);

	originalPoints = vector<cVector3d>(mesh->m_vertices->getNumElements());
	forces = vector<cVector3d>(mesh->m_vertices->getNumElements());
	velocities = vector<cVector3d>(mesh->m_vertices->getNumElements());

	for (unsigned int i = 0; i < mesh->m_vertices->getNumElements(); i++)
	{
		originalPoints[i] = mesh->m_vertices->getLocalPos(i);
		forces[i] = cVector3d(0, 0, 0);
		velocities[i] = cVector3d(0, 0, 0);
	}
}


MatchingMesh::~MatchingMesh()
{
}

void MatchingMesh::update(cVector3d &force, cVector3d &position, cVector3d &cursorPosition)
{
	forceComputed.assign(mesh->m_vertices->getNumElements(), 0);
	cVector3d collisionPoint = cVector3d(0,0,0);
	bool collided = false;
	float count = 0;


	affectedVertices.clear();
	bool lContact = contact;

	contact = false;
	for (unsigned int i = 0; i < mesh->getNumTriangles(); i++) {
		if (updateTriangle(force, position, cursorPosition, i))
		{
			contact = true;
		}
	}
	for (auto& e : edges) {
		if (updateLine(e.first, e.second, position, cursorPosition))
		{
			contact = true;
		}
	}
	for (unsigned int i = 0; i < mesh->m_vertices->getNumElements(); i++)
	{
		cVector3d cPos = cursorPosition;
		if (updatePoint(i, position, cursorPosition))
		{
			contact = true;
		}
	}
	if (contact && !lContact) {
		lCursorPosition = cursorPosition;
	}

	/*for (unsigned int i = 0; i < mesh->getNumTriangles(); i++) {
		cVector3d cPos = cursorPosition;
		if (updateTriangle(force, position, cPos, i))
		{
			collided = true;
			count += 1.f;
			collisionPoint += cPos;
		}
	}
	for (auto& e : edges) {
		cVector3d cPos = cursorPosition;
		if (updateLine(e.first, e.second, position, cPos))
		{
			count += 1.f;
			collided = true;
			collisionPoint += cPos;
		}
	}
	for (unsigned int i = 0; i < mesh->m_vertices->getNumElements(); i++)
	{
		cVector3d cPos = cursorPosition;
		if (updatePoint(i, position, cPos))
		{
			count += 1.f;
			collided = true;
			collisionPoint += cPos;
		}
	}

	if (collided)
		cursorPosition = (collisionPoint)/count;*/

	if (contact&&lContact) {
		auto d = cursorPosition - lCursorPosition;
		if (cDynamicFriction) {
			if (d.length() > dynamicFriction) {
				d.normalize();
				lCursorPosition = cursorPosition - d * (dynamicFriction+0.0001);
			}
			else {
				cDynamicFriction = 0;
			}
		}
		else {
			if (d.length() > staticFriction) {
				d.normalize();
				lCursorPosition = cursorPosition - d * (staticFriction+0.0001);
			}
		}

		cursorPosition = lCursorPosition;
	}

	force = (cursorPosition - position) * 2000;

	for (auto i : affectedVertices) {
		forces[i] += -force * 0.1;
	}

	for (unsigned int i = 0; i < mesh->m_vertices->getNumElements(); i++)
	{
		movePoint(i);
	}

	prevPos = cursorPosition;
}

float planeIntersection(cVector3d ray, cVector3d origin, cVector3d n, cVector3d q)
{
	n.normalize();
	if (cDot(ray, n) != 0)
		return (cDot(q, n) - cDot(n, origin)) / cDot(ray, n);

	return -1;
}

//candidate 1
cVector3d calc_barycentric_coords(cVector3d p0, cVector3d p1, cVector3d p2, cVector3d point)
{
	// calculating barycentric 
	cVector3d barycentric;
	{ auto d = p1 - p0; d.cross(point - p0); barycentric.z(d.length() * (d.z() / abs(d.z()))); }
	{ auto d = p2 - p1; d.cross(point - p1); barycentric.x(d.length() * (d.z() / abs(d.z()))); }
	{ auto d = p0 - p2; d.cross(point - p2); barycentric.y(d.length() * (d.z() / abs(d.z()))); }
	barycentric.normalize();

	return barycentric;
}

cVector3d triangleIntersection(cVector3d p0, cVector3d p1, cVector3d p2, cVector3d origin, cVector3d ray, float &t)
{
	cVector3d s = origin - p0;
	cVector3d e1 = p1 - p0;
	cVector3d e2 = p2 - p0;

	Matrix3f mt;// = mat3(s, e1, e2);
	mt << s.x(), e1.x(), e2.x(),  s.y(), e1.y(), e2.y(),  s.z(), e1.z(), e2.z();

	Matrix3f mu; //= mat3(-ray, s, e2);
	mu << -ray.x(), s.x(), e2.x(), -ray.y(), s.y(), e2.y(), -ray.z(), s.z(), e2.z();

	Matrix3f mv; // = mat3(-ray, e1, s);
	mv << -ray.x(), e1.x(), s.x(), -ray.y(), e1.y(), s.y(), -ray.z(), e1.z(), s.z() ;
	Matrix3f md;// = mat3(-ray, e1, e2);
	md << -ray.x(), e1.x(), e2.x(), -ray.y(), e1.y(), e2.y(), -ray.z(), e1.z(), e2.z();

	t = mt.determinant() / md.determinant();
	float u = mu.determinant() / md.determinant();
	float v = mv.determinant() / md.determinant();
	float w = 1 - u - v;

	return cVector3d(w,u,v);

}

bool barycentric_test(cVector3d bar)
{
	if (bar.x() >= 0 && bar.y() >= 0 && bar.z() >= 0)
	{
		return true;
	}

	return false;
}

bool MatchingMesh::updateTriangle(chai3d::cVector3d &force, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition, unsigned int index) {

	unsigned int index0 = (mesh->m_triangles->getVertexIndex0(index));
	cVector3d p0 = mesh->m_vertices->getLocalPos(index0);

	unsigned int index1 = (mesh->m_triangles->getVertexIndex1(index));
	cVector3d p1 = mesh->m_vertices->getLocalPos(index1);

	unsigned int index2 = (mesh->m_triangles->getVertexIndex2(index));
	cVector3d p2 = mesh->m_vertices->getLocalPos(index2);

	if (!forceComputed[index0])	forces[index0] = cVector3d(0, 0, 0);
	if (!forceComputed[index1]) forces[index1] = cVector3d(0, 0, 0);
	if (!forceComputed[index2])	forces[index2] = cVector3d(0, 0, 0);

	auto normal = (p1 - p0); normal.cross(p2 - p0); normal.normalize();
	// triangle between tool and avatar
	cVector3d toolToAvatar = prevPos - position;
	toolToAvatar.normalize();
	/*float t = planeIntersection(toolToAvatar, position, normal, p0);
	cVector3d intersection = position + toolToAvatar * t;
	cVector3d bar_intersection = calc_barycentric_coords(p0, p1, p2, intersection);*/

	float t;
	cVector3d bar_intersection = triangleIntersection(p0,p1,p2, position, toolToAvatar, t);
	if (t >= 0 && t <= (prevPos - position).length())
	{
		if (barycentric_test(bar_intersection))
		{
			cursorPosition = position + toolToAvatar * (t+0.1);
		}
	}

	bool contact = 0;
	auto proj = (cursorPosition - p0);
	auto proj_dot_normal = proj.dot(normal);
	auto proj_sign = (proj_dot_normal > 0) ? 1 : -1;
	proj = proj_dot_normal * normal;
	{
		// check cursor sphere against the face
		auto proj_face = cursorPosition - proj;

		//cVector3d barycentric = calc_barycentric_coords(p0,p1,p2, proj_face);

		cVector3d barycentric = triangleIntersection(p0,p1,p2, cursorPosition, cNormalize(cursorPosition-proj_face), t);

		// cursor sphere touching triangle face
		if (abs(proj_dot_normal) < radius && barycentric.x() >= 0 && barycentric.y() >= 0 && barycentric.z() >= 0
			&& barycentric.x() <= 1 && barycentric.y() <= 1 && barycentric.z() <= 1) {
			cursorPosition = (proj_face + normal * radius * proj_dot_normal / abs(proj_dot_normal));
			contact = 1;

			// direction * barycentric coordinate (weight) * penetration depth * sign * stiffness
			forces[index0] += -normal * barycentric.x() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
			forces[index1] += -normal * barycentric.y() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
			forces[index2] += -normal * barycentric.z() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
			forceComputed[index0] = forceComputed[index1] = forceComputed[index2] = 1;
		}
	}

	if (contact) {
		affectedVertices.insert(index0);
		affectedVertices.insert(index1);
		affectedVertices.insert(index2);
	}

	return contact;
}

cVector3d solveCylinderCollisions(cVector3d c1, cVector3d c2, float radius, cVector3d l1, cVector3d l2)
{
	cVector3d cylinderDir = c2 - c1;
	cylinderDir.normalize();
	cVector3d lineDir = l2-l1;
	lineDir.normalize();

	//calcuate vector from one line to the other
	cVector3d line_distance = l1-c1;
	//attemtp to get normal
	cVector3d normal = cCross(cylinderDir, lineDir);
	float distance = 0;
	//If no normal is found then the vectors are colinear
	if (normal.length() == 0)
	{
		//The following calculates the orthogonal direction from one parallel line onto the other
		cVector3d lineV = cProject(line_distance, lineDir);
		lineV = line_distance-lineV;
		distance = lineV.length();
	}
	else
	{
		//project line onto the normal (orthogonal to both lines)
		//This yields the orthogonal distance, the shortest distance between the lines
		line_distance = cProject(line_distance, normal);
		distance = line_distance.length();
	}

	if (distance > radius)
	{
		return c1;
	}

	cVector3d c1p = c1- line_distance;
	cVector3d c2p = c2 - line_distance;

	Matrix3f A;
	//Construct the matrix [v1, v2, 0]
	A << cylinderDir.x(), -lineDir.x(), 0, cylinderDir.y(), -lineDir.y(), 0, cylinderDir.z(), -lineDir.z(), 0;

	cVector3d offset = l2 - c1p;


	Vector3f b;
	//Create the result vector p2-p1
	b << offset.x(), offset.y(), offset.z();

	//Solve the system of equations
	Vector3f x = A.colPivHouseholderQr().solve(b);
	//cout << "test: "<< x << endl;

	bool inside = false;
	cVector3d c3 = c1 + cylinderDir * x[0];
	cVector3d l3 = l1 + lineDir * x[1];

	//cout << c1 + cylinderDir * x[0] << endl;
	return c1+cylinderDir*x[0];
}

bool MatchingMesh::updateLine(unsigned int indexP0, unsigned int indexP1, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition) {
	auto p0 = mesh->m_vertices->getLocalPos(indexP0);
	auto p1 = mesh->m_vertices->getLocalPos(indexP1);

	//cursorPosition = solveCylinderCollisions(prevPos, position, radius, p0, p1);

	bool contact = false;
	// cursor sphere touching line segment p0 p1
	auto d = cursorPosition - p0;
	auto l01 = p1 - p0;
	auto lerp_amount = d.dot(l01) / l01.dot(l01);
	auto proj = l01 * lerp_amount;

	if (lerp_amount >= 0 && lerp_amount <= 1) {
		auto o = d - proj;
		auto penetration_depth = radius - o.length();
		o.normalize();
		if (penetration_depth > 0) {
			contact = 1;
			// compute cursor position
			auto target_contact = p0 + proj;
			cursorPosition = (target_contact + o * radius);
			// apply forces to the triangle
			auto fd = -(cursorPosition - position); fd.normalize();
			forces[indexP0] += fd * (1 - lerp_amount) * penetration_depth * 1000;
			forces[indexP1] += fd * (lerp_amount)* penetration_depth * 1000;
		}
	}
	if (contact) {
		affectedVertices.insert(indexP0);
		affectedVertices.insert(indexP1);
	}
	return contact;
}

bool MatchingMesh::updatePoint(unsigned int index, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition) {
	bool contact = false;
	auto p0 = mesh->m_vertices->getLocalPos(index);

	auto d = cursorPosition - p0;
	auto f = d.length() - radius;
	if (f < 0) {
		d.normalize();
		cursorPosition = (p0 + d * (radius));
		contact = 1;
		auto fd = -(cursorPosition - position); fd.normalize();
		forces[index] = -f * fd * 1000;
	}
	if (contact) {
		affectedVertices.insert(index);
	}
	return contact;
}

void MatchingMesh::movePoint(unsigned int index)
{
	double delta = 0.001;
	auto p = mesh->m_vertices->getLocalPos(index);

	forces[index] += (originalPoints[index] - p) * k - velocities[index] * b;

	velocities[index] += forces[index] * delta / m;

	p += velocities[index] * delta;

	mesh->m_vertices->setLocalPos(index, p);
}