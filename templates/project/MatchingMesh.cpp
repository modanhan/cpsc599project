#include "MatchingMesh.h"
#include "loader.hpp"

using namespace std;
using namespace chai3d;

MatchingMesh::MatchingMesh(cWorld* world)
{
	object = new cMesh();
	mesh = new cMesh();

	vector<cVector3d> vertices;
	vector<cVector3d> normals;
	vector<cVector3d> uvs;
	vector<unsigned int> indices;

	loadOBJ("Meshes/monkey.obj", &vertices, &normals, &uvs, &indices);
	object->setEnabled(0);

	cout << vertices.size() << endl;
	cout << indices.size() << endl;
	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		mesh->newVertex(vertices[i], normals[i]);
	}
	mesh->setVertexColor(cColorf(1, 1, 1, 1));

	for (unsigned int i = 0; i < indices.size() / 3; i++)
	{
		mesh->newTriangle(indices[i * 3], indices[i * 3 + 1], indices[i * 3 + 2]);
	}
	mesh->computeAllNormals();

	mesh->m_material = cMaterial::create();
	mesh->m_material->setBlueMediumTurquoise();
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
	for (unsigned int i = 0; i < mesh->getNumTriangles(); i++) {
		updateTriangle(force, position, cursorPosition, i);
	}

	force = (cursorPosition - position) * 2000;

	for (unsigned int i = 0; i < mesh->m_vertices->getNumElements(); i++)
	{
		movePoint(i);
	}
}


bool MatchingMesh::updateTriangle(chai3d::cVector3d &force, chai3d::cVector3d &position, chai3d::cVector3d &cursorPosition, unsigned int index) {
	unsigned int index0 = (mesh->m_triangles->getVertexIndex0(index));
	cVector3d p0 = mesh->m_vertices->getLocalPos(index0);

	unsigned int index1 = (mesh->m_triangles->getVertexIndex1(index));
	cVector3d p1 = mesh->m_vertices->getLocalPos(index1);

	unsigned int index2 = (mesh->m_triangles->getVertexIndex2(index));
	cVector3d p2 = mesh->m_vertices->getLocalPos(index2);

	forces[index0] = forces[index1] = forces[index2] = cVector3d(0, 0, 0);

	bool contact = 0;
	auto normal = (p1 - p0); normal.cross(p2 - p0); normal.normalize();
	auto proj = (cursorPosition - p0);
	auto proj_dot_normal = proj.dot(normal);
	auto proj_sign = (proj_dot_normal > 0) ? 1 : -1;
	proj = proj_dot_normal * normal;
	{
		// check cursor sphere against the face
		auto proj_face = cursorPosition - proj;

		// calculating barycentric 
		cVector3d barycentric;
		{ auto d = p1 - p0; d.cross(proj_face - p0); barycentric.z(d.length() * (d.z() / abs(d.z()))); }
		{ auto d = p2 - p1; d.cross(proj_face - p1); barycentric.x(d.length() * (d.z() / abs(d.z()))); }
		{ auto d = p0 - p2; d.cross(proj_face - p2); barycentric.y(d.length() * (d.z() / abs(d.z()))); }
		barycentric.normalize();

		// cursor sphere touching triangle face
		if (abs(proj_dot_normal) < radius && barycentric.x() >= 0 && barycentric.y() >= 0 && barycentric.z() >= 0
			&& barycentric.x() <= 1 && barycentric.y() <= 1 && barycentric.z() <= 1) {
			cursorPosition = (proj_face + normal * radius * proj_dot_normal / abs(proj_dot_normal));
			contact = 1;

			// direction * barycentric coordinate (weight) * penetration depth * sign * stiffness
			forces[index0] = -normal * barycentric.x() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
			forces[index1] = -normal * barycentric.y() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
			forces[index2] = -normal * barycentric.z() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;

		}
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