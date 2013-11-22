/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef CONSTRAINT_DEMO_H
#define CONSTRAINT_DEMO_H

#include "OpenGL/GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication

class GNRR;

struct HandState {
	btScalar scale;

	btTransform basePose;
	btScalar spread;
	btScalar finger1;
	btScalar finger2;
	btScalar finger3;

	btScalar spreada;
	btScalar finger1a;
	btScalar finger2a;
	btScalar finger3a;

	HandState(const GNRR* gnrr);

	void update(const GNRR* gnrr);

	btScalar getJoint(int finger, int joint) const;
	btTransform getLinkFrame(int finger, int link) const;
	btTransform getFingertipPose(int finger) const;

	std::string toString() const;
};

class GNRR : public PlatformDemoApplication
{
	//keep track of variables to delete memory at the end
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	void	setupEmptyDynamicsWorld();

	void	clientResetScene();


	public:

	GNRR(float scale=10.);
	virtual ~GNRR();

	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	static DemoApplication* Create()
	{
		GNRR* demo = new GNRR();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	virtual void keyboardCallback(unsigned char key, int x, int y);

	bool run;

	// for cone-twist motor driving
	float m_time;

	btTransform getBasePose();
	btTransform getFingertipPose(int finger);
	
	void setBaseConstraintPose(const btTransform& pose, bool constrainAngles=false);
	void releaseBaseConstraint();
	void setFingerConstraintPose(int finger, const btTransform& pose, bool constrainAngles=false);

//	void 	enableSpring (int index, bool onOff)
//	void 	setStiffness (int index, btScalar stiffness)
//	void 	setDamping (int index, btScalar damping)
//	void 	setEquilibriumPoint ()
//	void 	setEquilibriumPoint (int index)
//	void 	setEquilibriumPoint (int index, btScalar val)

	bool inTimeRange(float start, float stop, float& value);

	float scale;
	float mass_exp;

	btRigidBody* handbase;
	btRigidBody* handbase_finger31;

	btRigidBody* finger11;
	btRigidBody* finger12;
	btRigidBody* finger13;

	btRigidBody* finger21;
	btRigidBody* finger22;
	btRigidBody* finger23;

	btRigidBody* finger32;
	btRigidBody* finger33;

	btGeneric6DofSpringConstraint* j_hb_11_jf4;
	btGeneric6DofSpringConstraint* j_hb_21_jf4mimic;
	btGearConstraint* jf4_gear;
	btGeneric6DofSpringConstraint* j_hb_31_fixed;

	btGeneric6DofSpringConstraint* j_11_12_jf1;
	btGeneric6DofSpringConstraint* j_12_13_jf1mimic;
	btGearConstraint* jf1_gear;

	btGeneric6DofSpringConstraint* j_21_22_jf2;
	btGeneric6DofSpringConstraint* j_22_23_jf2mimic;
	btGearConstraint* jf2_gear;

	btGeneric6DofSpringConstraint* j_31_32_jf3;
	btGeneric6DofSpringConstraint* j_32_33_jf3mimic;
	btGearConstraint* jf3_gear;

	btGeneric6DofSpringConstraint* p2p;

	btGeneric6DofSpringConstraint* baseConstraint;
	btGeneric6DofSpringConstraint* finger1Constraint;
	btGeneric6DofSpringConstraint* finger2Constraint;
	btGeneric6DofSpringConstraint* finger3Constraint;
};

class Mesh {
	btDynamicsWorld* m_dynamicsWorld;
};

class Gripper {
	btDynamicsWorld* m_dynamicsWorld;
};

#endif //CONSTRAINT_DEMO_H

