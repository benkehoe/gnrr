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


#include <iostream>
#include <sstream>

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btGearConstraint.h"
#include "LinearMath/btIDebugDraw.h"

#include "OpenGL/GLDebugDrawer.h"

#include "OpenGL/GLDebugFont.h"
#include <stdio.h> //printf debugging

#include "gnrr.h"
#include "OpenGL/GL_ShapeDrawer.h"
#include "OpenGL/GlutStuff.h"

#include "OpenGL/GLDebugDrawer.h"
static GLDebugDrawer	gDebugDrawer;

btScalar GNRR::scale = 10.;

inline btScalar interpolate(btScalar start, btScalar end, btScalar value) {
	return (end-start) * value + start;
}

inline btVector3 interpolate(const btVector3& start, const btVector3& end, btScalar value) {
	return (end-start) * value + start;
}

inline btQuaternion interpolate(const btQuaternion& start, const btQuaternion& end, btScalar value) {
	return start.slerp(end,value);
}

inline btMatrix3x3 interpolate(const btMatrix3x3& start, const btMatrix3x3& end, btScalar value) {
	btQuaternion qstart, qend;
	start.getRotation(qstart);
	end.getRotation(qend);
	return btMatrix3x3(interpolate(qstart,qend,value));
}

inline btTransform interpolate(const btTransform& start, const btTransform& end, btScalar value) {
	btVector3 p = interpolate(start.getOrigin(), end.getOrigin(), value);
	btMatrix3x3 r = interpolate(start.getBasis(), end.getBasis(), value);
	return btTransform(r,p);
}

inline JointState interpolate(const JointState& start, const JointState& end, btScalar value) {
	JointState interp;
	interp.spread = interpolate(start.spread, end.spread, value);
	interp.finger1 = interpolate(start.finger1, end.finger1, value);
	interp.finger2 = interpolate(start.finger2, end.finger2, value);
	interp.finger3 = interpolate(start.finger3, end.finger3, value);

	interp.spreada = interpolate(start.spreada, end.spreada, value);
	interp.finger1a = interpolate(start.finger1a, end.finger1a, value);
	interp.finger2a = interpolate(start.finger2a, end.finger2a, value);
	interp.finger3a = interpolate(start.finger3a, end.finger3a, value);

	return interp;
}

inline std::string toString(const btVector3& vector) {
	char s[50];
	sprintf(s, "(% .3f, % .3f, % .3f)", vector.getX(), vector.getY(), vector.getZ());
	return std::string(s);
}

inline std::string toString(const btMatrix3x3& m) {
	btScalar yaw, pitch, roll;
	m.getEulerZYX(roll, pitch, yaw);
	char s[50];
	sprintf(s, "[% 6.1f, % 6.1f, % 6.1f]", yaw * 180. / SIMD_PI, pitch * 180. / SIMD_PI, roll * 180. / SIMD_PI);
	return std::string(s);
}

inline std::string toString(const btQuaternion& quat) {
	return toString(btMatrix3x3(quat));
}

inline std::string toString(const btTransform& tf) {
	std::stringstream ss;
	ss << toString(tf.getOrigin()) << ", " << toString(tf.getBasis());
	return ss.str();
}

inline std::ostream& operator<<(std::ostream& o, const btVector3& obj) {
	o << toString(obj);
	return o;
}

inline std::ostream& operator<<(std::ostream& o, const btMatrix3x3& obj) {
	o << toString(obj);
	return o;
}

inline std::ostream& operator<<(std::ostream& o, const btQuaternion& obj) {
	o << toString(obj);
	return o;
}

inline std::ostream& operator<<(std::ostream& o, const btTransform& obj) {
	o << toString(obj);
	return o;
}

JointState::JointState() {
	spread = 0;
	finger1 = 0;
	finger2 = 0;
	finger3 = 0;

	spreada = 0;
	finger1a = 0;
	finger2a = 0;
	finger3a = 0;
}

JointState::JointState(const GNRR* gnrr) {
	update(gnrr);
}

void JointState::update(const GNRR* gnrr) {
	spread = gnrr->j_hb_11_jf4->getAngle(2);
	finger1 = gnrr->j_11_12_jf1->getAngle(2);
	finger2 = gnrr->j_21_22_jf2->getAngle(2);
	finger3 = gnrr->j_31_32_jf3->getAngle(2);

	spreada = SIMD_PI - gnrr->j_hb_21_jf4mimic->getAngle(2);
	finger1a = gnrr->j_12_13_jf1mimic->getAngle(2);
	finger2a = gnrr->j_22_23_jf2mimic->getAngle(2);
	finger3a = gnrr->j_32_33_jf3mimic->getAngle(2);
}

btScalar JointState::getValue(int finger, int joint) const {
	static btScalar nan = 1./0.;
	switch (finger) {
	case 1:
		switch (joint) {
		case 0:
			return spread;
		case 1:
			return finger1;
		case 2:
			return finger1a;
		default:
			return nan;
		}
	case 2:
		switch (joint) {
		case 0:
			return spreada;
		case 1:
			return finger2;
		case 2:
			return finger2a;
		default:
			return nan;
		}
	case 3:
		switch (joint) {
		case 0:
			return SIMD_PI;
		case 1:
			return finger3;
		case 2:
			return finger3a;
		default:
			return nan;
		}
	default:
		return nan;
	}
}

std::string JointState::toString(bool all) const {
	std::stringstream ss;
	if (all) {
		ss << spread << "/" << spreada << ", ";
		ss << finger1 << "/" << finger1a << ", ";
		ss << finger2 << "/" << finger2a << ", ";
		ss << finger3 << "/" << finger3a;
	} else {
		ss << spread << ", ";
		ss << finger1 << ", ";
		ss << finger2 << ", ";
		ss << finger3;
	}
	return ss.str();
}
inline std::ostream& operator<<(std::ostream& o, const JointState& obj) {
	o << obj.toString();
	return o;
}

HandState::HandState(const btTransform& theBasePose, const JointState& jointState) :
		basePose(theBasePose), joints(jointState) {
}

HandState::HandState(const GNRR* gnrr) {
	update(gnrr);
}

void HandState::update(const GNRR* gnrr) {
	basePose = gnrr->handbase->getCenterOfMassTransform();
	joints.update(gnrr);
}

btScalar HandState::getJointValue(int finger, int joint) const {
	return joints.getValue(finger, joint);
}

btTransform HandState::getLinkFrame(int finger, int link) const {
	if (link == 0) {
		return btTransform::getIdentity();
	} else if (link == 4) {
		return getFingertipPose(finger);
	}

	btTransform baseFrame = getLinkFrame(finger, link-1);

	btTransform linkFrame;
	linkFrame.setIdentity();
	switch (link) {
	case 1:
		switch (finger) {
		case 1:
			linkFrame.setOrigin(btVector3(0, GNRR::scale* -0.025, GNRR::scale*0.0415));
			break;
		case 2:
			linkFrame.setOrigin(btVector3(0, GNRR::scale*  0.025, GNRR::scale*0.0415));
			break;
		case 3:
			linkFrame.setOrigin(btVector3(0,             0, GNRR::scale*0.0415));
			break;
		}
		break;
	case 2:
		linkFrame.setOrigin(btVector3(GNRR::scale*0.050, 0, GNRR::scale*0.034));
		linkFrame.getBasis().setEulerZYX(SIMD_PI_2, 0, 0);
		break;
	case 3:
		linkFrame.setOrigin(btVector3(GNRR::scale*0.070, 0, 0));
		break;
	}

	btScalar jointValue = getJointValue(finger, link-1);

	btTransform joint;
	joint.setIdentity();
	joint.getBasis().setEulerZYX(0,0,jointValue);

	return baseFrame * linkFrame * joint;
}

btTransform HandState::getFingertipPose(int finger) const {
	btTransform fingertip;
	fingertip.setOrigin(btVector3(GNRR::scale*0.058,0,0));
	fingertip.getBasis().setEulerZYX(SIMD_PI_2, 0, 0);
	return getLinkFrame(finger, 3) * fingertip;
}

std::string HandState::toString() const {
	std::stringstream ss;
	ss << "Base pose:" << std::endl;
	ss << "  " << ::toString(basePose.getOrigin()) << std::endl;
	ss << "  " << ::toString(basePose.getRotation()) << std::endl;
	ss << "Joints:" << std::endl;
	ss << "  " << joints.spread * 180. / SIMD_PI << ", "
			<< joints.finger1 * 180. / SIMD_PI << ", "
			<< joints.finger2 * 180. / SIMD_PI << ", "
			<< joints.finger3 * 180. / SIMD_PI;
	return ss.str();
}
inline std::ostream& operator<<(std::ostream& o, const HandState& obj) {
	o << obj.toString();
	return o;
}

JointConstraint::JointConstraint(btRigidBody& rA, btRigidBody& rB,
		const btTransform& frameInA, const btTransform& frameInB,
		btScalar theLowerLimit, btScalar theUpperLimit) :
		btGeneric6DofSpringConstraint(rA, rB, frameInA, frameInB, true), lowerLimit(theLowerLimit), upperLimit(theUpperLimit) {

	setLinearUpperLimit(btVector3(0., 0., 0.));
	setLinearLowerLimit(btVector3(0., 0., 0.));

	setAngularLowerLimit(btVector3(0.f, 0.f, lowerLimit));
	setAngularUpperLimit(btVector3(0.f, 0.f, upperLimit));
}

btScalar JointConstraint::getAngle() {
	return getAngle(2);
}

btScalar JointConstraint::getAngle(int index) {
	return ((btGeneric6DofSpringConstraint*)this)->getAngle(index);
}

void JointConstraint::setAngle(btScalar angle) {
	if (isSprung()) {
		setEquilibriumPoint(5, angle);
	} else {
		btVector3 lower, upper;
		getAngularLowerLimit(lower);
		getAngularUpperLimit(upper);
		lower.setZ(angle);
		upper.setZ(angle);
		setAngularLowerLimit(lower);
		setAngularUpperLimit(upper);
	}
}

void JointConstraint::setFree() {
	setAngularLowerLimit(btVector3(0.f, 0.f, lowerLimit));
	setAngularUpperLimit(btVector3(0.f, 0.f, upperLimit));
}

void JointConstraint::enableSpring(bool on) {
	if (!on) {
		disableSpring();
		return;
	}
	((btGeneric6DofSpringConstraint*)this)->enableSpring(5, true);
	setStiffness(5, 1);
	setDamping(5, 5);

	setFree();
}

void JointConstraint::disableSpring() {
	if (!springEnabled(5)) {
		return;
	}

	((btGeneric6DofSpringConstraint*)this)->enableSpring(5, false);
	setAngle(getEquilibriumPoint(5));
}

bool JointConstraint::isSprung() {
	for (int i=0; i<6; i++) {
		if (m_springEnabled[i]) {
			return true;
		}
	}
	return false;
}

bool JointConstraint::springEnabled(int index) {
	return m_springEnabled[index];
}
bool JointConstraint::getStiffness(int index) {
	return m_springStiffness[index];
}
bool JointConstraint::getDamping(int index) {
	return m_springDamping[index];
}
btScalar JointConstraint::getEquilibriumPoint(int index) {
	return m_equilibriumPoint[index];
}

ContactConstraint::ContactConstraint(btRigidBody& rb, const btTransform& frame) :
		btGeneric6DofSpringConstraint(rb, frame, true) {
	setLinearLowerLimit(btVector3(0.,0.,0.));
	setLinearUpperLimit(btVector3(-0.00001f,-0.00001f,-0.00001f));
}

btVector3 ContactConstraint::getPoint() {
	btVector3 lower, upper, avg;
	getLinearLowerLimit(lower);
	getLinearUpperLimit(upper);
	avg = (lower + upper) / 2;
	return avg;
}

void ContactConstraint::setPoint(const btVector3& point) {
	btScalar fudge = GNRR::scale * .03;
	btVector3 lower = point;
	btVector3 upper = point;
	if (isSprung()) {
		lower += btVector3(fudge,fudge,0);
		upper -= btVector3(fudge,fudge,0);
	}
	setLinearLowerLimit(lower);
	setLinearUpperLimit(upper);
}

void ContactConstraint::setFree() {
	btScalar fudge = GNRR::scale * .03;
	btVector3 lower, upper, avg;
	getLinearLowerLimit(lower);
	getLinearUpperLimit(upper);

	avg = (lower + upper) / 2;
	lower = avg;
	upper = avg;

	lower += btVector3(fudge,fudge,0);
	upper -= btVector3(fudge,fudge,0);

	setLinearLowerLimit(lower);
	setLinearUpperLimit(upper);
}

void ContactConstraint::enableSpring(bool on) {
	if (!on) {
		disableSpring();
		return;
	}
	for (int i=0; i<2; i++) {
		((btGeneric6DofSpringConstraint*)this)->enableSpring(i, true);
		setStiffness(i, GNRR::scale * 1);
		setDamping(i, GNRR::scale * .5);
	}

	setFree();
}

void ContactConstraint::disableSpring() {
	for (int i=0;i<6;i++) {
		((btGeneric6DofSpringConstraint*)this)->enableSpring(i, false);

		btVector3 lower, upper, avg;
		getLinearLowerLimit(lower);
		getLinearUpperLimit(upper);
		avg = (lower + upper) / 2;

		setLinearLowerLimit(avg);
		setLinearUpperLimit(avg);
	}
}

bool ContactConstraint::isSprung() {
	for (int i=0; i<6; i++) {
		if (m_springEnabled[i]) {
			return true;
		}
	}
	return false;
}

bool ContactConstraint::springEnabled(int index) {
	return m_springEnabled[index];
}
bool ContactConstraint::getStiffness(int index) {
	return m_springStiffness[index];
}
bool ContactConstraint::getDamping(int index) {
	return m_springDamping[index];
}
btScalar ContactConstraint::getEquilibriumPoint(int index) {
	return m_equilibriumPoint[index];
}

GNRR::GNRR() {
	run = false;

	mass_exp = 2;

	handbase =0;
	handbase_finger31 =0;

	finger11 =0;
	finger12 =0;
	finger13 =0;

	finger21 =0;
	finger22 =0;
	finger23 =0;

	finger32 =0;
	finger33 =0;

	j_hb_11_jf4 = 0;
	j_hb_21_jf4mimic = 0;
	jf4_gear = 0;
	j_hb_31_fixed = 0;

	j_11_12_jf1 = 0;
	j_12_13_jf1mimic = 0;
	jf1_gear = 0;

	j_21_22_jf2 = 0;
	j_22_23_jf2mimic = 0;
	jf2_gear = 0;

	j_31_32_jf3 = 0;
	j_32_33_jf3mimic = 0;
	jf3_gear = 0;

	p2p = 0;

	baseConstraint = 0;
	finger1Constraint = 0;
	finger2Constraint = 0;
	finger3Constraint = 0;
}

/*
void	drawLimit()
{
		btVector3 from = sliderTransform*lowerSliderLimit;
		btVector3 to = sliderTransform*hiSliderLimit;
		btVector3 color(255,0,0);
		glBegin(GL_LINES);
		glColor3f(color.getX(), color.getY(), color.getZ());
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		if (d6body0)
		{
			from = d6body0->getWorldTransform().getOrigin();
			to = from + d6body0->getWorldTransform().getBasis() * btVector3(0,0,10);
			glVertex3d(from.getX(), from.getY(), from.getZ());
			glVertex3d(to.getX(), to.getY(), to.getZ());
		}
		glEnd();
}
*/


void GNRR::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

//	drawLimit();

	renderme();

    glFlush();
    swapBuffers();
}


void GNRR::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;
	switch (key)
	{
	case 'a' :
	{
		btVector3 lower;
		p2p->getLinearLowerLimit(lower);
		btVector3 upper;
		p2p->getLinearUpperLimit(upper);
		lower.setX(lower.getX() + scale *( 0.001));
		upper.setX(lower.getX() + scale *( 0.001));
		p2p->setLinearLowerLimit(lower);
		p2p->setLinearUpperLimit(upper);
	}
	break;
	case 'd' :
	{
		btVector3 lower;
		p2p->getLinearLowerLimit(lower);
		btVector3 upper;
		p2p->getLinearUpperLimit(upper);
		lower.setX(lower.getX() - scale *( 0.001));
		upper.setX(lower.getX() - scale *( 0.001));
		p2p->setLinearLowerLimit(lower);
		p2p->setLinearUpperLimit(upper);
	}
	break;
	case 'w' :
	{
		btVector3 lower;
		p2p->getLinearLowerLimit(lower);
		btVector3 upper;
		p2p->getLinearUpperLimit(upper);
		lower.setY(lower.getY() + scale *( 0.001));
		upper.setY(lower.getY() + scale *( 0.001));
		p2p->setLinearLowerLimit(lower);
		p2p->setLinearUpperLimit(upper);
	}
	break;
	case 's' :
	{
		btVector3 lower;
		p2p->getLinearLowerLimit(lower);
		btVector3 upper;
		p2p->getLinearUpperLimit(upper);
		lower.setY(lower.getY() - scale *( 0.001));
		upper.setY(lower.getY() - scale *( 0.001));
		p2p->setLinearLowerLimit(lower);
		p2p->setLinearUpperLimit(upper);
	}
	break;
	case ' ':
	{
		run = !run;
	}
	break;
	default :
	{
		DemoApplication::keyboardCallback(key, x, y);
	}
	break;
	}
}

void	GNRR::setupEmptyDynamicsWorld()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_overlappingPairCache = new btDbvtBroadphase();
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);

}

void	GNRR::clientResetScene()
{
	exitPhysics();
	initPhysics();
}
void	GNRR::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(5.f);
	m_time = 0;

	setupEmptyDynamicsWorld();

	m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);
	
	m_dynamicsWorld->setGravity(btVector3(0.,0.,0.));

	//float mass = 1.f;
	
	btTransform tr;
	btTransform childTransform;

	/*************************** links ***************************/

	float fingerX1_width = 0.01;
	btCollisionShape* fingerX1_shape = new btBoxShape(btVector3(scale*0.05/2, scale*fingerX1_width/2, scale*0.034/2));
	m_collisionShapes.push_back(fingerX1_shape);

	/***************** handbase ******************/

	btCompoundShape* handbase_shape = new btCompoundShape();
	m_collisionShapes.push_back(handbase_shape);

	childTransform.setIdentity();
	childTransform.setOrigin(btVector3(0,0,scale*0.0415/2));
	handbase_shape->addChildShape(childTransform,new btCylinderShapeZ(btVector3(scale*0.089/2,scale*0.09/2,scale*0.0415/2)));

	childTransform.setIdentity();
	childTransform.setOrigin(btVector3(scale*-0.05/2, 0, scale*(0.0415 +0.034/2)));
	handbase_shape->addChildShape(childTransform,fingerX1_shape);

	tr.setIdentity();
	tr.setOrigin(btVector3(0., 0., 0.));
	tr.getBasis().setEulerZYX(0,0,0);
	handbase = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, handbase_shape);
	handbase->setActivationState(DISABLE_DEACTIVATION);

	/***************** fingerX1 ******************/

	/******** finger11 *********/

	btCompoundShape* finger11_shape = new btCompoundShape();
	m_collisionShapes.push_back(finger11_shape);

	childTransform.setIdentity();
	childTransform.setOrigin(btVector3(scale*0.05/2, 0, scale*0.034/2));
	finger11_shape->addChildShape(childTransform,fingerX1_shape);

	tr.setIdentity();
	//tr.setOrigin(btVector3(scale*0.05/2, scale* -0.025, scale*(0.0415 + 0.034/2)));
	tr.setOrigin(btVector3(0, scale* -0.025, scale*0.0415));
	tr.getBasis().setEulerZYX(0,0,0);

	finger11 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, finger11_shape);
	if (finger11) finger11->setActivationState(DISABLE_DEACTIVATION);

	/******** finger12 *********/

	btCompoundShape* finger12_shape = new btCompoundShape();
	m_collisionShapes.push_back(finger12_shape);

	childTransform.setIdentity();
	childTransform.setOrigin(btVector3(scale*0.05/2, 0, scale*0.034/2));
	finger12_shape->addChildShape(childTransform,fingerX1_shape);

	tr.setIdentity();
	//tr.setOrigin(btVector3(scale*0.05/2, scale*0.025, scale*(0.0415 + 0.034/2)));
	tr.setOrigin(btVector3(0, scale*0.025, scale*(0.0415)));
	tr.getBasis().setEulerZYX(0,0,0);

	finger21 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, finger12_shape);
	if (finger21) finger21->setActivationState(DISABLE_DEACTIVATION);

	//finger31 - in handbase compound shape


	/***************** fingerX2 ******************/

	float fingerX2_radius = 0.01;
	btCollisionShape* fingerX2_capsule_shape = new btCapsuleShapeX(scale * fingerX2_radius, scale * (0.07 - 2 * fingerX2_radius));
	m_collisionShapes.push_back(fingerX2_capsule_shape);
	btCompoundShape* fingerX2_shape = new btCompoundShape();
	m_collisionShapes.push_back(fingerX2_shape);

	childTransform.setIdentity();
	childTransform.setOrigin(btVector3(scale*0.07/2, 0, 0));
	fingerX2_shape->addChildShape(childTransform,fingerX2_capsule_shape);


	/******** finger12 *********/

	tr.setIdentity();
	//tr.setOrigin(btVector3(scale*(0.05 + 0.07/2), -scale*0.025, scale*(0.0415 + 0.034)));
	tr.setOrigin(btVector3(scale*0.05, -scale*0.025, scale*(0.0415 + 0.034)));
	//tr.setBasis(btMatrix3x3(1,0,0,0,0,-1,0,1,0));
	tr.getBasis().setEulerZYX(-SIMD_PI_2,0,0);

	finger12 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX2_shape);
	if (finger12) finger12->setActivationState(DISABLE_DEACTIVATION);

	/******** finger22 *********/

	tr.setIdentity();
	//tr.setOrigin(btVector3(scale*(0.05 + 0.07/2), scale*0.025, scale*(0.0415 + 0.034)));
	tr.setOrigin(btVector3(scale*0.05, scale*0.025, scale*(0.0415 + 0.034)));
	//tr.setBasis(btMatrix3x3(1,0,0,0,0,-1,0,1,0));
	tr.getBasis().setEulerZYX(-SIMD_PI_2,0,0);

	finger22 = localCreateRigidBody( 0.1, tr, fingerX2_shape);
	if (finger22) finger22->setActivationState(DISABLE_DEACTIVATION);

	/******** finger32 *********/

	tr.setIdentity();
	//tr.setOrigin(btVector3(-scale*(0.05 + 0.07/2), 0., scale*(0.0415 + 0.034)));
	tr.setOrigin(btVector3(-scale*0.05, 0., scale*(0.0415 + 0.034)));
	tr.setBasis(btMatrix3x3(-1,0,0,0,0,1,0,1,0));

	finger32 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX2_shape);
	if (finger32) finger32->setActivationState(DISABLE_DEACTIVATION);

	/***************** fingerX3 ******************/

	float fingerX3_radius = 0.01;
	btCollisionShape* fingerX3_capsule_shape = new btCapsuleShapeX(scale * fingerX3_radius, scale * (0.058 - 2 * fingerX3_radius));
	m_collisionShapes.push_back(fingerX3_capsule_shape);
	btCompoundShape* fingerX3_shape = new btCompoundShape();
	m_collisionShapes.push_back(fingerX3_shape);

	childTransform.setIdentity();
	childTransform.setOrigin(btVector3(scale*0.058/2, 0, 0));
	fingerX3_shape->addChildShape(childTransform,fingerX3_capsule_shape);

	btMatrix3x3 basis;
	btScalar adjustAngle = 0.8727;
	btMatrix3x3 adjust = btMatrix3x3::getIdentity();
	adjust.setEulerZYX(0,0,-adjustAngle);

	/******** finger13 *********/

	tr.setIdentity();
	//tr.setOrigin(btVector3(scale*(0.05 + 0.07 + (0.058/2)*cos(adjustAngle)), -scale*0.025, scale*(0.0415 + 0.034 + (0.058/2)*sin(adjustAngle))));
	tr.setOrigin(btVector3(scale*(0.05 + 0.07), -scale*0.025, scale*(0.0415 + 0.034)));
	//basis = btMatrix3x3(1,0,0,0,0,-1,0,1,0);
	basis.setEulerZYX(-SIMD_PI_2,0,0);
	basis *= adjust;
	tr.setBasis(basis);

	finger13 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX3_shape);
	if (finger13) finger13->setActivationState(DISABLE_DEACTIVATION);

	/******** finger23 *********/

	tr.setIdentity();
	tr.setOrigin(btVector3(scale*(0.05 + 0.07), scale*0.025, scale*(0.0415 + 0.034)));
	//basis = btMatrix3x3(1,0,0,0,0,-1,0,1,0);
	basis.setEulerZYX(-SIMD_PI_2,0,0);
	basis *= adjust;
	tr.setBasis(basis);

	finger23 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX3_shape);
	if (finger23) finger23->setActivationState(DISABLE_DEACTIVATION);

	/******** finger33 *********/

	adjust.setEulerZYX(0,0,adjustAngle);

	tr.setIdentity();
	tr.setOrigin(btVector3(-scale*(0.05 + 0.07), 0., scale*(0.0415 + 0.034)));
	basis = btMatrix3x3(-1,0,0,0,0,1,0,1,0);
	basis *= adjust;
	tr.setBasis(basis);

	finger33 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX3_shape);
	if (finger33) finger33->setActivationState(DISABLE_DEACTIVATION);

	/*************************** constraints ***************************/

	btTransform frameInA, frameInB;

	/***************** handbase-X0 ******************/

	/******** handbase-11 *********/

	if (finger11 && true) {
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(0,scale*-0.025,scale*0.0415));
	frameInA.getBasis().setEulerZYX(0,0,SIMD_PI);
	frameInB.setIdentity();
	//frameInB.setOrigin(btVector3(scale * -0.05/2,0,scale* -0.034/2));
	frameInB.getBasis().setEulerZYX(0,0,SIMD_PI);

	j_hb_11_jf4 = new JointConstraint(*handbase,*finger11,frameInA,frameInB,
			-1. * SIMD_PI / 180., 180. * SIMD_PI / 180.);

	m_dynamicsWorld->addConstraint(j_hb_11_jf4, true);
	//j_hb_11_jf4->setDbgDrawSize(btScalar(5.f));
	}

	/******** handbase-21 *********/

	if (finger21 && true) {
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(0,scale*0.025,scale*0.0415));
	//frameInA.getBasis().setEulerZYX(0,0,SIMD_PI);
	frameInB.setIdentity();
	//frameInB.setOrigin(btVector3(scale * -0.05/2,0,scale* -0.034/2));
	frameInB.getBasis().setEulerZYX(0,0,SIMD_PI);

	j_hb_21_jf4mimic = new JointConstraint(*handbase,*finger21,frameInA,frameInB,
			-1. * SIMD_PI / 180., 180. * SIMD_PI / 180.);

	m_dynamicsWorld->addConstraint(j_hb_21_jf4mimic, true);
	//j_hb_21_jf4mimic->setDbgDrawSize(btScalar(5.f));
	}

	/******** 11-21 gear ********/

	if (finger11 && finger21 && true) {
	jf4_gear = new btGearConstraint(*finger11,*finger21,btVector3(0,0,1),btVector3(0,0,1),1);
	m_dynamicsWorld->addConstraint(jf4_gear, true);
	jf4_gear->setDbgDrawSize(10.);
	}

	/***************** X1-X2 ******************/

	/******** 11-12 *********/

	if (finger11 && finger12 && true) {
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(scale*0.05,0,scale * 0.034));
	//frameInA.setBasis(btMatrix3x3(1,0,0,0,0,-1,0,1,0));
	frameInA.getBasis().setEulerZYX(-SIMD_PI_2,0,0);
	frameInB.setIdentity();
	//frameInB.setOrigin(btVector3(scale * -0.07/2,0,0));
	frameInB.getBasis().setEulerZYX(0,0,0);

	j_11_12_jf1 = new JointConstraint(*finger11,*finger12,frameInA,frameInB,
			0. * SIMD_PI / 180., 140. * SIMD_PI / 180.);

	m_dynamicsWorld->addConstraint(j_11_12_jf1, true);
	//j_11_12_jf1->setDbgDrawSize(btScalar(5.f));
	}

	/******** 21-22 *********/

	if (finger21 && finger22 && true) {
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(scale*0.05,0,scale * 0.034));
	//frameInA.setBasis(btMatrix3x3(1,0,0,0,0,-1,0,1,0));
	frameInA.getBasis().setEulerZYX(-SIMD_PI_2,0,0);
	frameInB.setIdentity();
	//frameInB.setOrigin(btVector3(scale * -0.07/2,0,0));
	frameInB.getBasis().setEulerZYX(0,0,0);

	j_21_22_jf2 = new JointConstraint(*finger21,*finger22,frameInA,frameInB,
			0. * SIMD_PI / 180., 140. * SIMD_PI / 180.);

	m_dynamicsWorld->addConstraint(j_21_22_jf2, true);
	//j_31_22_jf2->setDbgDrawSize(btScalar(5.f));
	}

	/******** handbase-32 *********/

	if (finger32 && true) {
	frameInA.setIdentity();
	//frameInA.setOrigin(btVector3(scale* -0.05,0,scale * (0.0415/2 + 0.034)));
	frameInA.setOrigin(btVector3(scale* -0.05,0,scale * (0.0415 + 0.034)));
	//frameInA.setBasis(btMatrix3x3(-1,0,0,0,0,1,0,1,0));
	frameInA.getBasis().setEulerZYX(-SIMD_PI_2,0,SIMD_PI);
	frameInB.setIdentity();
	//frameInB.setOrigin(btVector3(scale * -0.07/2,0,0));
	frameInB.getBasis().setEulerZYX(SIMD_PI,0,0);

	j_31_32_jf3 = new JointConstraint(*handbase,*finger32,frameInA,frameInB,
			0. * SIMD_PI / 180., 140. * SIMD_PI / 180.);

	m_dynamicsWorld->addConstraint(j_31_32_jf3, true);
	//j_31_32_jf3->setDbgDrawSize(btScalar(5.f));
	}

	/***************** X2-X3 ******************/

	/******** 12-13 *********/

	if (finger12 && finger13 && true) {
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(scale*0.07,0,0));
	frameInB.setIdentity();
	//frameInB.setOrigin(btVector3(scale * -0.058/2,0,0));

	j_12_13_jf1mimic = new JointConstraint(*finger12,*finger13,frameInA,frameInB,
			50. * SIMD_PI / 180., 97. * SIMD_PI / 180.);

	m_dynamicsWorld->addConstraint(j_12_13_jf1mimic, true);
	//j_12_13_jf1mimic->setDbgDrawSize(btScalar(5.f));
	}

	/******** 12-13 gear ********/

	if (finger12 && finger13 && true) {
	jf1_gear = new btGearConstraint(*finger12,*finger13,btVector3(0,0,1),btVector3(0,0,1),-0.7);
	m_dynamicsWorld->addConstraint(jf1_gear, true);
	}

	/******** 22-23 *********/

	if (finger22 && finger23 && true) {
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(scale*0.07,0,0));
	frameInB.setIdentity();
	//frameInB.setOrigin(btVector3(scale * -0.058/2,0,0));

	j_22_23_jf2mimic = new JointConstraint(*finger22,*finger23,frameInA,frameInB,
			50. * SIMD_PI / 180., 97. * SIMD_PI / 180.);

	m_dynamicsWorld->addConstraint(j_22_23_jf2mimic, true);
	//j_11_12_jf2mimic->setDbgDrawSize(btScalar(5.f));
	}

	/******** 22-23 gear ********/

	if (finger22 && finger23 && true) {
	jf2_gear = new btGearConstraint(*finger22,*finger23,btVector3(0,0,1),btVector3(0,0,1),-0.7);
	m_dynamicsWorld->addConstraint(jf2_gear, true);
	}

	/******** 32-33 *********/

	if (finger32 && finger33 && true) {
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(scale*0.07,0,0));
	frameInA.getBasis().setEulerZYX(SIMD_PI,0,0);
	frameInB.setIdentity();
	//frameInB.setOrigin(btVector3(scale * -0.058/2,0,0));
	frameInB.getBasis().setEulerZYX(SIMD_PI,0,0);

	j_32_33_jf3mimic = new JointConstraint(*finger32,*finger33,frameInA,frameInB,
			50. * SIMD_PI / 180., 97. * SIMD_PI / 180.);

	m_dynamicsWorld->addConstraint(j_32_33_jf3mimic, true);
	//j_21_22_jf3mimic->setDbgDrawSize(btScalar(5.f));
	}

	/******** 32-33 gear ********/

	if (finger32 && finger33 && true) {
	jf3_gear = new btGearConstraint(*finger32,*finger33,btVector3(0,0,1),btVector3(0,0,1),-0.7);
	m_dynamicsWorld->addConstraint(jf3_gear, true);
	}

	if (handbase) {
		frameInB.setIdentity();
		baseConstraint = new ContactConstraint(*handbase,frameInB);
		m_dynamicsWorld->addConstraint(baseConstraint, true);
	}

	if (finger13) {
		frameInB.setIdentity();
		frameInB.setOrigin(btVector3(scale*0.058,0,0));
		finger1Constraint = new ContactConstraint(*finger13,frameInB);
		m_dynamicsWorld->addConstraint(finger1Constraint, true);
	}

	if (finger23) {
		frameInB.setIdentity();
		frameInB.setOrigin(btVector3(scale*0.058,0,0));
		finger2Constraint = new ContactConstraint(*finger23,frameInB);
		m_dynamicsWorld->addConstraint(finger2Constraint, true);
	}

	if (finger33) {
		frameInB.setIdentity();
		frameInB.setOrigin(btVector3(scale*0.058,0,0));
		finger3Constraint = new ContactConstraint(*finger33,frameInB);
		m_dynamicsWorld->addConstraint(finger3Constraint, true);
	}

	HandState state = getState();
	setupConstraints(state);
}

void	GNRR::exitPhysics()
{

		int i;

	//removed/delete constraints
	for (i=m_dynamicsWorld->getNumConstraints()-1; i>=0 ;i--)
	{
		btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
		m_dynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}

	//remove the rigidbodies from the dynamics world and delete them
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}




	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	m_collisionShapes.clear();

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

}

GNRR::~GNRR()
{
	//cleanup in the reverse order of creation/initialization

	exitPhysics();

}

HandState GNRR::getState() {
	return HandState(this);
}

JointState GNRR::getJointState() {
	return JointState(this);
}

void GNRR::setBaseConstraintPose(const btTransform& pose, bool constrainAngles) {
	if (handbase && baseConstraint) {
		btTransform frameInB = btTransform::getIdentity();
		btTransform frameInA = handbase->getCenterOfMassTransform() * frameInB;
		baseConstraint->setFrames(frameInA, frameInB);

		btTransform poseTranformed = frameInA.inverseTimes(pose);

		btVector3 pt = poseTranformed.getOrigin();
		//printf("%f %f %f\n",pt.getX(), pt.getY(), pt.getZ());

		baseConstraint->setLinearLowerLimit(btVector3(pt.x(),pt.y(),pt.z()));
		baseConstraint->setLinearUpperLimit(btVector3(pt.x(),pt.y(),pt.z()));

		if (constrainAngles) {
			btScalar yaw;
			btScalar pitch;
			btScalar roll;
			poseTranformed.getBasis().getEulerYPR(yaw,pitch,roll);

			//printf("%f %f %f\n",yaw,pitch,roll);

			baseConstraint->setAngularLowerLimit(btVector3(yaw,pitch,roll));
			baseConstraint->setAngularUpperLimit(btVector3(yaw,pitch,roll));
		}

	}
}
void GNRR::unsetBaseConstraint() {
	if (handbase && baseConstraint) {
		btTransform frameInB = btTransform::getIdentity();
		btTransform frameInA = handbase->getCenterOfMassTransform() * frameInB;
		baseConstraint->setFrames(frameInA, frameInB);

		baseConstraint->setLinearLowerLimit(btVector3(0.,0.,0.));
		baseConstraint->setLinearUpperLimit(btVector3(-0.00001f,-0.00001f,-0.00001f));

		baseConstraint->setAngularLowerLimit(btVector3(0.,0.,0.));
		baseConstraint->setAngularUpperLimit(btVector3(-0.00001f,-0.00001f,-0.00001f));
	}
}

void GNRR::setFingerConstraintPose(int fingerNum, const btTransform& pose, btScalar angle, bool constrainAngles) {
	btTransform fingertip;
	fingertip.setOrigin(btVector3(scale*0.058,0,0));
	fingertip.getBasis().setEulerZYX(SIMD_PI_2, 0, 0);

	btTransform fingerAngle;
	fingerAngle.setIdentity();
	fingerAngle.getBasis().setEulerZYX(0,-angle,0);

	btRigidBody* fingerBody = 0;
	ContactConstraint* constraint = 0;
	switch (fingerNum) {
	case 1:
		fingerBody = finger13;
		constraint = finger1Constraint;
		break;
	case 2:
		fingerBody = finger23;
		constraint = finger2Constraint;
		break;
	case 3:
		fingerBody = finger33;
		constraint = finger3Constraint;
		break;
	}
	if (fingerBody && constraint) {
		btTransform frameInB = fingertip;
		btTransform frameInA = fingerBody->getCenterOfMassTransform() * fingertip;
		constraint->setFrames(frameInA, frameInB);

		btTransform poseTranformed = frameInA.inverseTimes(pose);

		btVector3 pt = poseTranformed.getOrigin();
		//printf("%f %f %f\n",pt.getX(), pt.getY(), pt.getZ());

		constraint->setPoint(pt);

		if (constrainAngles) {
			btScalar yaw;
			btScalar pitch;
			btScalar roll;
			poseTranformed.getBasis().getEulerYPR(yaw,pitch,roll);

			//printf("%f %f %f\n",yaw,pitch,roll);

			constraint->setAngularLowerLimit(btVector3(yaw,pitch,roll));
			constraint->setAngularUpperLimit(btVector3(yaw,pitch,roll));
		}
	}
}

void GNRR::setJointConstraints(JointState& jointState, bool spread) {
	if (spread) {
		getJointConstraint(1,0)->enableSpring();
		getJointConstraint(1,0)->setAngle(jointState.getValue(1,0));
	}

	for (int i=1; i<=3; i++) {
		getJointConstraint(i,1)->enableSpring();
		getJointConstraint(i,1)->setAngle(jointState.getValue(i,1));
	}
}

void GNRR::unsetJointConstraints() {
	for (int i=1; i<=3; i++) {
		for (int j=0; j<=2; j++) {
			JointConstraint* constraint = getJointConstraint(i,j);
			if (constraint) {
				constraint->disableSpring();
			}
		}
	}
}

bool GNRR::inTimeRange(float start, float stop, float& value) {
	if (!run || m_time <= start || stop < m_time) {
		return false;
	} else {
		value = (m_time-start) / (stop-start);
		return true;
	}
}

void GNRR::addStage(stageCallback callback, float duration, const std::string& name) {
	Stage stage;
	stage.name = name;
	stage.duration = duration;
	stage.callback = callback;
	stages.push_back(stage);
}

void GNRR::clientMoveAndDisplay()
{
	
 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

 	float dt = float(getDeltaTimeMicroseconds()) * 0.000001f;
	//printf("dt = %f: ",dt);

 	if (run) {
 		m_time += 0.03f;
 	}

	static bool once = true;
	if (once) {
		once = false;

	}

	if (run) {
		//printf("%f %f\n", state.getJointValue(1,1), getJointConstraint(1,1)->getAngle());
	}

	if (run && !stages.empty()) {
		static size_t prevStageNum = -1;
		float stageStart = 0;
		float value;
		for (size_t stageNum=0; stageNum<=stages.size(); stageNum++) {
			if (stageNum == stages.size()) {
				if (prevStageNum == stages.size()-1) {
					printf("Stages finished\n");
				}
				prevStageNum = stageNum;
				break;
			}
			Stage stage = stages[stageNum];
			float stageEnd = stageStart + stage.duration;
			if (inTimeRange(stageStart, stageEnd, value)) {
				bool first = stageNum != prevStageNum;
				if (first) {
					if (stage.name.empty()) {
						printf("Entering stage %lu\n", stageNum);
					} else {
						printf("Entering stage %lu: %s\n", stageNum, stage.name.c_str());
					}
				}

				stage.callback(this, value, first);

				prevStageNum = stageNum;
				break;
			}
			stageStart = stageEnd;
		}
	}

	{
		static bool once = true;
		if ( m_dynamicsWorld->getDebugDrawer() && once)
		{
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
			once=false;
		}
	}

	
	{
	 	//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			dt = 1.0f/420.f / 10;


		int numSimSteps = 1;
		if (run) {
			numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
		}

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	
		bool verbose = false;
		if (verbose)
		{
			if (!numSimSteps)
				printf("Interpolated transforms\n");
			else
			{
				if (numSimSteps > maxSimSubSteps)
				{
					//detect dropping frames
					printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
				} else
				{
					printf("Simulated (%i) steps\n",numSimSteps);
				}
			}
		}
	}
	renderme();

//	drawLimit();

    glFlush();
    swapBuffers();
}

void GNRR::setupConstraints(HandState& state) {
	setFingerConstraintPose(1,state.getFingertipPose(1));
	setFingerConstraintPose(2,state.getFingertipPose(2));
	setFingerConstraintPose(3,state.getFingertipPose(3));

	ContactConstraint* constraint = getContactConstraint(1);

	constraint->enableSpring();

	setJointConstraints(state.joints);

//	for (int i=1; i<=3; i++) {
//		getJointConstraint(i, 1)->enableSpring();
////		getJointConstraint(i, 2)->enableSpring();
//	}
}

void test(GNRR* gnrr, float value, bool first) {
	static btTransform origFingerPose[3];
	static btTransform finalFingerPose[3];
	if (first) {
		for (int i=0; i<3; i++) {
			origFingerPose[i] = gnrr->getState().getFingertipPose(i+1);

			finalFingerPose[i].setIdentity();
			finalFingerPose[i].getOrigin().setZ(origFingerPose[i].getOrigin().getZ());
			finalFingerPose[i].getOrigin().setX(origFingerPose[i].getOrigin().getX()/2);
			finalFingerPose[i].getOrigin().setY(origFingerPose[i].getOrigin().getY() * 2);
		}
	}

	for (int i=0; i<3; i++) {
		gnrr->setFingerConstraintPose(i+1,interpolate(origFingerPose[i],finalFingerPose[i],value));
	}
}

void doNothing(GNRR* gnrr, float value, bool first) {

}

int main(int argc,char** argv)
{
	//Load meshes

	//init grasp

    GNRR* gnrr = new GNRR();
    gnrr->initPhysics();
	gnrr->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);

	gnrr->addStage(&test, 5, "test stage");

	return glutmain(argc, argv,640,480,"GNRR",gnrr);
}
