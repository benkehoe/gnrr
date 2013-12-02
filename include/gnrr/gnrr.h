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

#define SIMD_PI_2 ((SIMD_PI)*0.5f)
#define SIMD_PI_4 ((SIMD_PI)*0.25f)

class GNRR;

struct JointState {
	JointState();
	JointState(const GNRR* gnrr);
	void update(const GNRR* gnrr);

	btScalar getValue(int finger, int joint) const;

	btScalar spread;
	btScalar finger1;
	btScalar finger2;
	btScalar finger3;

	btScalar spreada;
	btScalar finger1a;
	btScalar finger2a;
	btScalar finger3a;

	std::string toString(bool all=false) const;
};

struct HandState {
	btTransform basePose;

	JointState joints;

	HandState(const GNRR* gnrr);
	void update(const GNRR* gnrr);

	btScalar getJointValue(int finger, int joint) const;
	btTransform getLinkFrame(int finger, int link) const;
	btTransform getFingertipPose(int finger) const;

	std::string toString() const;
};

class JointConstraint : public btGeneric6DofSpringConstraint {
public:
	JointConstraint(btRigidBody& rA, btRigidBody& rB, const btTransform& frameInA, const btTransform& frameInB);

	btScalar getAngle();
	btScalar getAngle(int index);
	void setAngle(btScalar angle);

	void enableSpring();
	void disableSpring();
	bool isSprung();

	bool springEnabled(int index);
	bool getStiffness(int index);
	bool getDamping(int index);
	btScalar getEquilibriumPoint(int index);
};

class ContactConstraint : public btGeneric6DofSpringConstraint {
public:
	ContactConstraint(btRigidBody& rb, const btTransform& frame);

	btVector3 getPoint();
	void setPoint(const btVector3& point);

	void enableSpring();
	void disableSpring();
	bool isSprung();

	bool springEnabled(int index);
	bool getStiffness(int index);
	bool getDamping(int index);
	btScalar getEquilibriumPoint(int index);
};

class GNRR : public PlatformDemoApplication
{

	//keep track of variables to delete memory at the end
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface* m_overlappingPairCache;

	class btCollisionDispatcher* m_dispatcher;

	class btConstraintSolver* m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	void setupEmptyDynamicsWorld();

	void clientResetScene();

	void setupConstraints(HandState& state);

public:

	static btScalar scale;
	GNRR();
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

	HandState getState();
	
	void setBaseConstraintPose(const btTransform& pose, bool constrainAngles=false);
	void releaseBaseConstraint();
	void setFingerConstraintPose(int finger, const btTransform& pose, btScalar angle=0, bool constrainAngles=false);

//	void 	enableSpring (int index, bool onOff)
//	void 	setStiffness (int index, btScalar stiffness)
//	void 	setDamping (int index, btScalar damping)
//	void 	setEquilibriumPoint ()
//	void 	setEquilibriumPoint (int index)
//	void 	setEquilibriumPoint (int index, btScalar val)

	bool inTimeRange(float start, float stop, float& value);

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

	JointConstraint* j_hb_11_jf4;
	JointConstraint* j_hb_21_jf4mimic;
	btGearConstraint* jf4_gear;
	JointConstraint* j_hb_31_fixed;

	JointConstraint* j_11_12_jf1;
	JointConstraint* j_12_13_jf1mimic;
	btGearConstraint* jf1_gear;

	JointConstraint* j_21_22_jf2;
	JointConstraint* j_22_23_jf2mimic;
	btGearConstraint* jf2_gear;

	JointConstraint* j_31_32_jf3;
	JointConstraint* j_32_33_jf3mimic;
	btGearConstraint* jf3_gear;

	btGeneric6DofSpringConstraint* p2p;

	ContactConstraint* baseConstraint;
	ContactConstraint* finger1Constraint;
	ContactConstraint* finger2Constraint;
	ContactConstraint* finger3Constraint;

	inline btRigidBody* getLink(int finger, int link) {
		switch (finger) {
		case 1:
			switch (link) {
			case 1:
				return finger11;
			case 2:
				return finger12;
			case 3:
				return finger13;
			default:
				return 0;
			}
		case 2:
			switch (link) {
			case 1:
				return finger21;
			case 2:
				return finger22;
			case 3:
				return finger23;
			default:
				return 0;
			}
		case 3:
			switch (link) {
			case 1:
				return 0;
			case 2:
				return finger32;
			case 3:
				return finger33;
			default:
				return 0;
			}
		default:
			return 0;
		}
	}

	inline JointConstraint* getJointConstraint(int finger, int joint) {
			switch (finger) {
			case 1:
				switch (joint) {
				case 0:
					return j_hb_11_jf4;
				case 1:
					return j_11_12_jf1;
				case 2:
					return j_12_13_jf1mimic;
				default:
					return 0;
				}
			case 2:
				switch (joint) {
				case 0:
					return j_hb_21_jf4mimic;
				case 1:
					return j_21_22_jf2;
				case 2:
					return j_22_23_jf2mimic;
				default:
					return 0;
				}
			case 3:
				switch (joint) {
				case 0:
					return 0;
				case 1:
					return j_31_32_jf3;
				case 2:
					return j_32_33_jf3mimic;
				default:
					return 0;
				}
			default:
				return 0;
			}
		}

	inline ContactConstraint* getContactConstraint(int finger) {
		switch (finger) {
		case 1:
			return finger1Constraint;
		case 2:
			return finger2Constraint;
		case 3:
			return finger3Constraint;
		default:
			return 0;
		}
	}
};

class Mesh {
	btDynamicsWorld* m_dynamicsWorld;
};

class Gripper {
	btDynamicsWorld* m_dynamicsWorld;
};

#endif //CONSTRAINT_DEMO_H

