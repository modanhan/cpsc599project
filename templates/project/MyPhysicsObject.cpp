#include "MyPhysicsObject.h"

using namespace chai3d;

MyPhysicsObject::MyPhysicsObject(cGenericObject* a_object)
{
	m_object = a_object;
	m_force = cVector3d(0, 0, 0);
	m_velocity = cVector3d(0, 0, 0);
}


MyPhysicsObject::~MyPhysicsObject()
{
}
