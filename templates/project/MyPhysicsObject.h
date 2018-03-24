#pragma once

#include "chai3d.h"

/*
	some super class the simulation of a particle; the transform comes from m_object, which is a cGenericObject, the mass comes from m_mass, a double
	has some basic functions for updating velocity and position
*/
class MyPhysicsObject
{
public:
	chai3d::cGenericObject* m_object;

	double m_mass;
	chai3d::cVector3d m_force;
	chai3d::cVector3d m_velocity;

	MyPhysicsObject(chai3d::cGenericObject* a_object);
	~MyPhysicsObject();

	virtual void UpdateForce(double delta) {}
	virtual void UpdateContactForce(double delta, chai3d::cVector3d cursor, chai3d::cVector3d& force) {}
	virtual void UpdateVelocity(double delta) { m_velocity += m_force * delta / m_mass; }
	virtual void UpdatePosition(double delta) { m_object->setLocalPos(m_object->getLocalPos() + m_velocity * delta); }

	void Update(double delta, chai3d::cVector3d cursor, chai3d::cVector3d& force) {
		if (!m_object->getEnabled())return;
		UpdateForce(delta);
		UpdateContactForce(delta, cursor, force);
		UpdateVelocity(delta);
		UpdatePosition(delta);
	};
};


/*
	affected by gravity and bounces back up when it hits a ground plane at z = 0
*/
class MyGravityObject : public MyPhysicsObject {

public:
	MyGravityObject(chai3d::cGenericObject* a_object) : MyPhysicsObject(a_object) {}

	void UpdateForce(double delta) override {
		m_force = chai3d::cVector3d(0, 0, -9.81) * m_mass;
		if (m_object->getLocalPos().z() < 0)m_force.z(0);
	}

	void UpdateVelocity(double delta) override {
		MyPhysicsObject::UpdateVelocity(delta);
		if (m_object->getLocalPos().z() < 0)m_velocity *= -.9;
	}

	void UpdatePosition(double delta) override {
		MyPhysicsObject::UpdatePosition(delta);
		if (m_object->getLocalPos().z() < 0)m_object->getLocalPos().z(0);
	}
};


/*
	anchored at a position with a spring, can be interacted with the avatar as a sphere
*/
class MyShapeMachingObject : public MyPhysicsObject {
public:
	chai3d::cVector3d m_anchor;
	double m_spring = 10;
	double m_damper = 0.5;

	MyShapeMachingObject(chai3d::cGenericObject* a_object, chai3d::cVector3d a_anchor) : MyPhysicsObject(a_object) { m_anchor = a_anchor; m_object->setLocalPos(m_anchor); }

	void UpdateForce(double delta) override {
		m_velocity += m_force * delta / m_mass;
		m_force = (m_anchor - m_object->getLocalPos()) * m_spring;
		m_force -= m_velocity * m_damper;
	}

	//void UpdateVelocity(double delta) override {}

	void UpdateContactForce(double delta, chai3d::cVector3d cursor, chai3d::cVector3d& force)  override {
		// terribly hard coded value
		double penetration = 0.02 - (cursor - m_object->getLocalPos()).length();
		if (penetration > 0) {
			auto contact = (cursor - m_object->getLocalPos());
			contact.normalize();
			contact *= penetration * 1000;
			m_force -= contact;
			force += contact;
		}
	}
};

class MyShapeMatchTriangle {

public:
	MyShapeMatchTriangle() {

	}

	~MyShapeMatchTriangle() {

	}

	void Update(double delta, chai3d::cVector3d cursor, chai3d::cVector3d& force) {

	}
};