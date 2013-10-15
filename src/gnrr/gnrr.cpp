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



#define SIMD_PI_2 ((SIMD_PI)*0.5f)
#define SIMD_PI_4 ((SIMD_PI)*0.25f)




float scale = 10.;
float mass_exp = 1;

btRigidBody* handbase =0;
btRigidBody* handbase_finger20 =0;

btRigidBody* finger00 =0;
btRigidBody* finger01 =0;
btRigidBody* finger02 =0;

btRigidBody* finger10 =0;
btRigidBody* finger11 =0;
btRigidBody* finger12 =0;

btRigidBody* finger21 =0;
btRigidBody* finger22 =0;

btGeneric6DofSpringConstraint* j_hb_00_jf4 = 0;
btGeneric6DofSpringConstraint* j_hb_10_jf4mimic = 0;
btGearConstraint* jf4_gear = 0;
btGeneric6DofSpringConstraint* j_hb_20_fixed = 0;

btGeneric6DofSpringConstraint* j_00_01_jf1 = 0;
btGeneric6DofSpringConstraint* j_01_02_jf1mimic = 0;
btGearConstraint* jf1_gear = 0;

btGeneric6DofSpringConstraint* j_10_11_jf2 = 0;
btGeneric6DofSpringConstraint* j_11_12_jf2mimic = 0;
btGearConstraint* jf2_gear = 0;

btGeneric6DofSpringConstraint* j_20_21_jf3 = 0;
btGeneric6DofSpringConstraint* j_21_22_jf3mimic = 0;
btGearConstraint* jf3_gear = 0;

btGeneric6DofSpringConstraint* p2p = 0;

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
	m_Time = 0;

	setupEmptyDynamicsWorld();

	m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);
	
	m_dynamicsWorld->setGravity(btVector3(0.,0.,0.));


	/*
	//btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(40.),btScalar(50.)));
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),40);

	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-56,0));
	btRigidBody* groundBody;
	groundBody= localCreateRigidBody(0, groundTransform, groundShape);



	*/

	float mass = 1.f;
	
	if(true){
		btTransform tr;
		btTransform childTransform;
		
		/*************************** links ***************************/
		
		float fingerX0_width = 0.01;
		btCollisionShape* fingerX0_shape = new btBoxShape(btVector3(scale*0.05/2, scale*fingerX0_width/2, scale*0.034/2));
		m_collisionShapes.push_back(fingerX0_shape);
		
		/***************** handbase ******************/
		
		btCompoundShape* handbase_shape = new btCompoundShape();
		m_collisionShapes.push_back(handbase_shape);
		
		childTransform.setIdentity();
		childTransform.setOrigin(btVector3(0,0,scale*0.0415/2));
		handbase_shape->addChildShape(childTransform,new btCylinderShapeZ(btVector3(scale*0.089/2,scale*0.09/2,scale*0.0415/2)));
		
		childTransform.setIdentity();
		childTransform.setOrigin(btVector3(scale*-0.05/2, 0, scale*(0.0415 +0.034/2)));
		handbase_shape->addChildShape(childTransform,fingerX0_shape);
		
		tr.setIdentity();
		tr.setOrigin(btVector3(0., 0., 0.));
		tr.getBasis().setEulerZYX(0,0,0);
		handbase = localCreateRigidBody( pow(scale,mass_exp)*0.0, tr, handbase_shape);
		handbase->setActivationState(DISABLE_DEACTIVATION);
		
		/***************** fingerX0 ******************/
		
		/******** finger00 *********/
		
		btCompoundShape* finger00_shape = new btCompoundShape();
		m_collisionShapes.push_back(finger00_shape);
		
		childTransform.setIdentity();
		childTransform.setOrigin(btVector3(scale*0.05/2, 0, scale*0.034/2));
		finger00_shape->addChildShape(childTransform,fingerX0_shape);
		
		tr.setIdentity();
		//tr.setOrigin(btVector3(scale*0.05/2, scale* -0.025, scale*(0.0415 + 0.034/2)));
		tr.setOrigin(btVector3(0, scale* -0.025, scale*0.0415));
		tr.getBasis().setEulerZYX(0,0,0);
		
		finger00 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, finger00_shape);
		if (finger00) finger00->setActivationState(DISABLE_DEACTIVATION);
		
		/******** finger10 *********/
		
		btCompoundShape* finger10_shape = new btCompoundShape();
		m_collisionShapes.push_back(finger10_shape);
		
		childTransform.setIdentity();
		childTransform.setOrigin(btVector3(scale*0.05/2, 0, scale*0.034/2));
		finger10_shape->addChildShape(childTransform,fingerX0_shape);
		
		tr.setIdentity();
		//tr.setOrigin(btVector3(scale*0.05/2, scale*0.025, scale*(0.0415 + 0.034/2)));
		tr.setOrigin(btVector3(0, scale*0.025, scale*(0.0415)));
		tr.getBasis().setEulerZYX(0,0,0);
		
		finger10 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, finger10_shape);
		if (finger10) finger10->setActivationState(DISABLE_DEACTIVATION);
		
		/******** "finger20" *********/
		
		//Using compound shape now
		/*
		tr.setIdentity();
		tr.setOrigin(btVector3(-scale*0.05/2, 0, scale*(0.0415 + 0.034/2)));
		tr.getBasis().setEulerZYX(0,0,0);
		
		handbase_finger20 = localCreateRigidBody( 0.1, tr, fingerX0_shape);
		if (handbase_finger20) handbase_finger20->setActivationState(DISABLE_DEACTIVATION);
		*/
		
		/***************** fingerX1 ******************/
		
		float fingerX1_radius = 0.01;
		btCollisionShape* fingerX1_capsule_shape = new btCapsuleShapeX(scale * fingerX1_radius, scale * (0.07 - 2 * fingerX1_radius));
		m_collisionShapes.push_back(fingerX1_capsule_shape);
		btCompoundShape* fingerX1_shape = new btCompoundShape();
		m_collisionShapes.push_back(fingerX1_shape);
		
		childTransform.setIdentity();
		childTransform.setOrigin(btVector3(scale*0.07/2, 0, 0));
		fingerX1_shape->addChildShape(childTransform,fingerX1_capsule_shape);
		
		
		/******** finger01 *********/
		
		tr.setIdentity();
		//tr.setOrigin(btVector3(scale*(0.05 + 0.07/2), -scale*0.025, scale*(0.0415 + 0.034)));
		tr.setOrigin(btVector3(scale*0.05, -scale*0.025, scale*(0.0415 + 0.034)));
		//tr.setBasis(btMatrix3x3(1,0,0,0,0,-1,0,1,0));
		tr.getBasis().setEulerZYX(-SIMD_PI_2,0,0);
		
		finger01 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX1_shape);
		if (finger01) finger01->setActivationState(DISABLE_DEACTIVATION);
		
		/******** finger11 *********/
		
		tr.setIdentity();
		//tr.setOrigin(btVector3(scale*(0.05 + 0.07/2), scale*0.025, scale*(0.0415 + 0.034)));
		tr.setOrigin(btVector3(scale*0.05, scale*0.025, scale*(0.0415 + 0.034)));
		//tr.setBasis(btMatrix3x3(1,0,0,0,0,-1,0,1,0));
		tr.getBasis().setEulerZYX(-SIMD_PI_2,0,0);
		
		finger11 = localCreateRigidBody( 0.1, tr, fingerX1_shape);
		if (finger11) finger11->setActivationState(DISABLE_DEACTIVATION);
		
		/******** finger21 *********/
		
		tr.setIdentity();
		//tr.setOrigin(btVector3(-scale*(0.05 + 0.07/2), 0., scale*(0.0415 + 0.034)));
		tr.setOrigin(btVector3(-scale*0.05, 0., scale*(0.0415 + 0.034)));
		tr.setBasis(btMatrix3x3(-1,0,0,0,0,1,0,1,0));
		
		finger21 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX1_shape);
		if (finger21) finger21->setActivationState(DISABLE_DEACTIVATION);
		
		/***************** fingerX2 ******************/
		
		float fingerX2_radius = 0.01;
		btCollisionShape* fingerX2_capsule_shape = new btCapsuleShapeX(scale * fingerX2_radius, scale * (0.058 - 2 * fingerX2_radius));
		m_collisionShapes.push_back(fingerX2_capsule_shape);
		btCompoundShape* fingerX2_shape = new btCompoundShape();
		m_collisionShapes.push_back(fingerX2_shape);
		
		childTransform.setIdentity();
		childTransform.setOrigin(btVector3(scale*0.058/2, 0, 0));
		fingerX2_shape->addChildShape(childTransform,fingerX2_capsule_shape);
		
		btMatrix3x3 basis;
		btScalar adjustAngle = 0.8727;
		btMatrix3x3 adjust = btMatrix3x3::getIdentity();
		adjust.setEulerZYX(0,0,-adjustAngle);
		
		/******** finger02 *********/
		
		tr.setIdentity();
		//tr.setOrigin(btVector3(scale*(0.05 + 0.07 + (0.058/2)*cos(adjustAngle)), -scale*0.025, scale*(0.0415 + 0.034 + (0.058/2)*sin(adjustAngle))));
		tr.setOrigin(btVector3(scale*(0.05 + 0.07), -scale*0.025, scale*(0.0415 + 0.034)));
		//basis = btMatrix3x3(1,0,0,0,0,-1,0,1,0);
		basis.setEulerZYX(-SIMD_PI_2,0,0);
		basis *= adjust;
		tr.setBasis(basis);
		
		finger02 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX2_shape);
		if (finger02) finger02->setActivationState(DISABLE_DEACTIVATION);
		
		/******** finger12 *********/
		
		tr.setIdentity();
		tr.setOrigin(btVector3(scale*(0.05 + 0.07), scale*0.025, scale*(0.0415 + 0.034)));
		//basis = btMatrix3x3(1,0,0,0,0,-1,0,1,0);
		basis.setEulerZYX(-SIMD_PI_2,0,0);
		basis *= adjust;
		tr.setBasis(basis);
		
		finger12 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX2_shape);
		if (finger12) finger12->setActivationState(DISABLE_DEACTIVATION);
		
		/******** finger22 *********/
		
		adjust.setEulerZYX(0,0,adjustAngle);
		
		tr.setIdentity();
		tr.setOrigin(btVector3(-scale*(0.05 + 0.07), 0., scale*(0.0415 + 0.034)));
		basis = btMatrix3x3(-1,0,0,0,0,1,0,1,0);
		basis *= adjust;
		tr.setBasis(basis);
		
		finger22 = localCreateRigidBody( pow(scale,mass_exp)*0.1, tr, fingerX2_shape);
		if (finger22) finger22->setActivationState(DISABLE_DEACTIVATION);
		
		/*************************** constraints ***************************/
		
		btTransform frameInA, frameInB;
		
		/***************** handbase-X0 ******************/
		
		/******** handbase-00 *********/
		
		if (finger00 && true) {
		frameInA.setIdentity();
		frameInA.setOrigin(btVector3(0,scale*-0.025,scale*0.0415));
		frameInA.getBasis().setEulerZYX(0,0,SIMD_PI);
		frameInB.setIdentity();
		//frameInB.setOrigin(btVector3(scale * -0.05/2,0,scale* -0.034/2));
		frameInB.getBasis().setEulerZYX(0,0,SIMD_PI);
		
		j_hb_00_jf4 = new btGeneric6DofSpringConstraint(*handbase,*finger00,frameInA,frameInB,true);
		j_hb_00_jf4->setLinearUpperLimit(btVector3(0., 0., 0.));
		j_hb_00_jf4->setLinearLowerLimit(btVector3(0., 0., 0.));

		j_hb_00_jf4->setAngularLowerLimit(btVector3(0.f, 0.f, -1. * SIMD_PI / 180.));
		j_hb_00_jf4->setAngularUpperLimit(btVector3(0.f, 0.f, 180. * SIMD_PI / 180.));
		
		m_dynamicsWorld->addConstraint(j_hb_00_jf4, true);
		//j_hb_00_jf4->setDbgDrawSize(btScalar(5.f));
		}
		
		/******** handbase-10 *********/
		
		if (finger10 && true) {
		frameInA.setIdentity();
		frameInA.setOrigin(btVector3(0,scale*0.025,scale*0.0415));
		//frameInA.getBasis().setEulerZYX(0,0,SIMD_PI);
		frameInB.setIdentity();
		//frameInB.setOrigin(btVector3(scale * -0.05/2,0,scale* -0.034/2));
		frameInB.getBasis().setEulerZYX(0,0,SIMD_PI);
		
		j_hb_10_jf4mimic = new btGeneric6DofSpringConstraint(*handbase,*finger10,frameInA,frameInB,true);
		j_hb_10_jf4mimic->setLinearUpperLimit(btVector3(0., 0., 0.));
		j_hb_10_jf4mimic->setLinearLowerLimit(btVector3(0., 0., 0.));

		j_hb_10_jf4mimic->setAngularLowerLimit(btVector3(0.f, 0.f, -1. * SIMD_PI / 180.));
		j_hb_10_jf4mimic->setAngularUpperLimit(btVector3(0.f, 0.f, 180. * SIMD_PI / 180.));
		
		m_dynamicsWorld->addConstraint(j_hb_10_jf4mimic, true);
		//j_hb_10_jf4mimic->setDbgDrawSize(btScalar(5.f));
		}
		
		/******** 00-10 gear ********/
		
		if (finger00 && finger01 && true) {
		jf4_gear = new btGearConstraint(*finger00,*finger10,btVector3(0,0,1),btVector3(0,0,1),1);
		m_dynamicsWorld->addConstraint(jf4_gear, true);
		jf4_gear->setDbgDrawSize(10.);
		}
		
		/******** handbase-"20" *********/
		
		//Using compound shape now
		/*
		frameInA.setIdentity();
		frameInA.setOrigin(btVector3(0,0,scale*0.0415/2));
		frameInA.getBasis().setEulerZYX(SIMD_PI,0,0);
		frameInB.setIdentity();
		frameInB.setOrigin(btVector3(scale * -0.05/2,0,scale* -0.034/2));
		
		j_hb_20_fixed = new btGeneric6DofSpringConstraint(*handbase,*handbase_finger20,frameInA,frameInB,true);
		j_hb_20_fixed->setLinearUpperLimit(btVector3(0., 0., 0.));
		j_hb_20_fixed->setLinearLowerLimit(btVector3(0., 0., 0.));

		j_hb_20_fixed->setAngularLowerLimit(btVector3(0.f, 0.f, 0));
		j_hb_20_fixed->setAngularUpperLimit(btVector3(0.f, 0.f, 0));
		
		m_dynamicsWorld->addConstraint(j_hb_20_fixed, true);
		//j_hb_20_fixed->setDbgDrawSize(btScalar(5.f));
		*/
		
		/***************** X0-X1 ******************/
		
		/******** 00-01 *********/
		
		if (finger00 && finger01 && true) {
		frameInA.setIdentity();
		frameInA.setOrigin(btVector3(scale*0.05,0,scale * 0.034));
		//frameInA.setBasis(btMatrix3x3(1,0,0,0,0,-1,0,1,0));
		frameInA.getBasis().setEulerZYX(-SIMD_PI_2,0,0);
		frameInB.setIdentity();
		//frameInB.setOrigin(btVector3(scale * -0.07/2,0,0));
		frameInB.getBasis().setEulerZYX(0,0,0);
		
		j_00_01_jf1 = new btGeneric6DofSpringConstraint(*finger00,*finger01,frameInA,frameInB,true);
		j_00_01_jf1->setLinearUpperLimit(btVector3(0., 0., 0.));
		j_00_01_jf1->setLinearLowerLimit(btVector3(0., 0., 0.));

		j_00_01_jf1->setAngularLowerLimit(btVector3(0.f, 0.f, 0. * SIMD_PI / 180.));
		j_00_01_jf1->setAngularUpperLimit(btVector3(0.f, 0.f, 140. * SIMD_PI / 180.));
		
		m_dynamicsWorld->addConstraint(j_00_01_jf1, true);
		//j_00_01_jf1->setDbgDrawSize(btScalar(5.f));
		}
		
		/******** 10-11 *********/
		
		if (finger10 && finger11 && true) {
		frameInA.setIdentity();
		frameInA.setOrigin(btVector3(scale*0.05,0,scale * 0.034));
		//frameInA.setBasis(btMatrix3x3(1,0,0,0,0,-1,0,1,0));
		frameInA.getBasis().setEulerZYX(-SIMD_PI_2,0,0);
		frameInB.setIdentity();
		//frameInB.setOrigin(btVector3(scale * -0.07/2,0,0));
		frameInB.getBasis().setEulerZYX(0,0,0);
		
		j_10_11_jf2 = new btGeneric6DofSpringConstraint(*finger10,*finger11,frameInA,frameInB,true);
		j_10_11_jf2->setLinearUpperLimit(btVector3(0., 0., 0.));
		j_10_11_jf2->setLinearLowerLimit(btVector3(0., 0., 0.));

		j_10_11_jf2->setAngularLowerLimit(btVector3(0.f, 0.f, 0. * SIMD_PI / 180.));
		j_10_11_jf2->setAngularUpperLimit(btVector3(0.f, 0.f, 140. * SIMD_PI / 180.));
		
		m_dynamicsWorld->addConstraint(j_10_11_jf2, true);
		//j_10_11_jf2->setDbgDrawSize(btScalar(5.f));
		}
		
		/******** handbase-21 *********/
		
		if (finger21 && true) {
		frameInA.setIdentity();
		//frameInA.setOrigin(btVector3(scale* -0.05,0,scale * (0.0415/2 + 0.034)));
		frameInA.setOrigin(btVector3(scale* -0.05,0,scale * (0.0415 + 0.034)));
		//frameInA.setBasis(btMatrix3x3(-1,0,0,0,0,1,0,1,0));
		frameInA.getBasis().setEulerZYX(-SIMD_PI_2,0,SIMD_PI);
		frameInB.setIdentity();
		//frameInB.setOrigin(btVector3(scale * -0.07/2,0,0));
		frameInB.getBasis().setEulerZYX(SIMD_PI,0,0);
		
		j_20_21_jf3 = new btGeneric6DofSpringConstraint(*handbase,*finger21,frameInA,frameInB,true);
		j_20_21_jf3->setLinearUpperLimit(btVector3(0., 0., 0.));
		j_20_21_jf3->setLinearLowerLimit(btVector3(0., 0., 0.));

		j_20_21_jf3->setAngularLowerLimit(btVector3(0.f, 0.f, 0. * SIMD_PI / 180.));
		j_20_21_jf3->setAngularUpperLimit(btVector3(0.f, 0.f, 140. * SIMD_PI / 180.));
		
		m_dynamicsWorld->addConstraint(j_20_21_jf3, true);
		//j_20_21_jf3->setDbgDrawSize(btScalar(5.f));
		}
		
		/***************** X1-X2 ******************/
		
		/******** 01-02 *********/
		
		if (finger01 && finger02 && true) {
		frameInA.setIdentity();
		frameInA.setOrigin(btVector3(scale*0.07,0,0));
		frameInB.setIdentity();
		//frameInB.setOrigin(btVector3(scale * -0.058/2,0,0));
		
		j_01_02_jf1mimic = new btGeneric6DofSpringConstraint(*finger01,*finger02,frameInA,frameInB,true);
		j_01_02_jf1mimic->setLinearUpperLimit(btVector3(0., 0., 0.));
		j_01_02_jf1mimic->setLinearLowerLimit(btVector3(0., 0., 0.));

		j_01_02_jf1mimic->setAngularLowerLimit(btVector3(0.f, 0.f, 50. * SIMD_PI / 180.));
		j_01_02_jf1mimic->setAngularUpperLimit(btVector3(0.f, 0.f, 97. * SIMD_PI / 180.));
		
		m_dynamicsWorld->addConstraint(j_01_02_jf1mimic, true);
		//j_01_02_jf1mimic->setDbgDrawSize(btScalar(5.f));
		}
		
		/******** 01-02 gear ********/
		
		if (finger01 && finger02 && true) {
		jf1_gear = new btGearConstraint(*finger01,*finger02,btVector3(0,0,1),btVector3(0,0,1),-0.7);
		m_dynamicsWorld->addConstraint(jf1_gear, true);
		}
		
		/******** 11-12 *********/
		
		if (finger11 && finger12 && true) {
		frameInA.setIdentity();
		frameInA.setOrigin(btVector3(scale*0.07,0,0));
		frameInB.setIdentity();
		//frameInB.setOrigin(btVector3(scale * -0.058/2,0,0));
		
		j_11_12_jf2mimic = new btGeneric6DofSpringConstraint(*finger11,*finger12,frameInA,frameInB,true);
		j_11_12_jf2mimic->setLinearUpperLimit(btVector3(0., 0., 0.));
		j_11_12_jf2mimic->setLinearLowerLimit(btVector3(0., 0., 0.));

		j_11_12_jf2mimic->setAngularLowerLimit(btVector3(0.f, 0.f, 50. * SIMD_PI / 180.));
		j_11_12_jf2mimic->setAngularUpperLimit(btVector3(0.f, 0.f, 97. * SIMD_PI / 180.));
		
		m_dynamicsWorld->addConstraint(j_11_12_jf2mimic, true);
		//j_11_12_jf2mimic->setDbgDrawSize(btScalar(5.f));
		}
		
		/******** 11-12 gear ********/
		
		if (finger11 && finger12 && true) {
		jf2_gear = new btGearConstraint(*finger11,*finger12,btVector3(0,0,1),btVector3(0,0,1),-0.7);
		m_dynamicsWorld->addConstraint(jf2_gear, true);
		}
		
		/******** 21-22 *********/
		
		if (finger21 && finger22 && true) {
		frameInA.setIdentity();
		frameInA.setOrigin(btVector3(scale*0.07,0,0));
		frameInA.getBasis().setEulerZYX(SIMD_PI,0,0);
		frameInB.setIdentity();
		//frameInB.setOrigin(btVector3(scale * -0.058/2,0,0));
		frameInB.getBasis().setEulerZYX(SIMD_PI,0,0);
		
		j_21_22_jf3mimic = new btGeneric6DofSpringConstraint(*finger21,*finger22,frameInA,frameInB,true);
		j_21_22_jf3mimic->setLinearUpperLimit(btVector3(0., 0., 0.));
		j_21_22_jf3mimic->setLinearLowerLimit(btVector3(0., 0., 0.));

		j_21_22_jf3mimic->setAngularLowerLimit(btVector3(0.f, 0.f, 50. * SIMD_PI / 180.));
		j_21_22_jf3mimic->setAngularUpperLimit(btVector3(0.f, 0.f, 97. * SIMD_PI / 180.));
		
		m_dynamicsWorld->addConstraint(j_21_22_jf3mimic, true);
		//j_21_22_jf3mimic->setDbgDrawSize(btScalar(5.f));
		}
		
		/******** 21-22 gear ********/
		
		if (finger21 && finger22 && true) {
		jf3_gear = new btGearConstraint(*finger21,*finger22,btVector3(0,0,1),btVector3(0,0,1),-0.7);
		m_dynamicsWorld->addConstraint(jf3_gear, true);
		}
		
		
		/******** p2p ********/
		if (true) {
		btVector3 pivot = (finger12->getCenterOfMassTransform() * btTransform(btMatrix3x3::getIdentity(),btVector3(scale*0.058,0,0))).invXform(btVector3(0,0,scale *0.120));
		printf("%f %f %f\n",pivot.getX(), pivot.getY(), pivot.getZ());
		frameInB.setIdentity();
		frameInB.setOrigin(btVector3(scale*0.058,0,0));
		
		p2p = new btGeneric6DofSpringConstraint(*finger12,frameInB,true);
		
		//p2p->setLinearLowerLimit(btVector3(-1,-1,-1));
		//p2p->setLinearUpperLimit(btVector3(1,1,1));
		p2p->setLinearLowerLimit(btVector3(pivot.getX(), pivot.getY(), pivot.getZ()));
		p2p->setLinearUpperLimit(btVector3(pivot.getX(), pivot.getY(), pivot.getZ()));
		
		//btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*finger22,pivot);
		m_dynamicsWorld->addConstraint(p2p, true);
		p2p->setDbgDrawSize(btScalar(5.f));
		}
		
	}
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


void GNRR::clientMoveAndDisplay()
{
	
 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

 	float dt = float(getDeltaTimeMicroseconds()) * 0.000001f;
	//printf("dt = %f: ",dt);

	// drive cone-twist motor
	m_Time += 0.03f;
	if (p2p) {
		btVector3 pt = finger22->getCenterOfMassTransform() * btVector3(scale*0.058,0,0);
		//printf("%f %f %f\n",pt.getX(), pt.getY(), pt.getZ());
		//printf("%f %f %f\n",p2p->getRelativePivotPosition(0),p2p->getRelativePivotPosition(1),p2p->getRelativePivotPosition(2));
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
		if (m_Time > 10) {
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
		case 'O' :
			{
				bool offectOnOff;
//				if(spDoorHinge)
//				{
//					offectOnOff = spDoorHinge->getUseFrameOffset();
//					offectOnOff = !offectOnOff;
//					spDoorHinge->setUseFrameOffset(offectOnOff);
//					printf("DoorHinge %s frame offset\n", offectOnOff ? "uses" : "does not use");
//				}
//				if(spHingeDynAB)
//				{
//					offectOnOff = spHingeDynAB->getUseFrameOffset();
//					offectOnOff = !offectOnOff;
//					spHingeDynAB->setUseFrameOffset(offectOnOff);
//					printf("HingeDynAB %s frame offset\n", offectOnOff ? "uses" : "does not use");
//				}
//				if(spSlider6Dof)
//				{
//					offectOnOff = spSlider6Dof->getUseFrameOffset();
//					offectOnOff = !offectOnOff;
//					spSlider6Dof->setUseFrameOffset(offectOnOff);
//					printf("Slider6Dof %s frame offset\n", offectOnOff ? "uses" : "does not use");
//				}
			}
			break;
		default : 
			{
				DemoApplication::keyboardCallback(key, x, y);
			}
			break;
	}
}

int main(int argc,char** argv)
{
	//Load meshes

	//init grasp

    GNRR* gnrr = new GNRR();


    gnrr->initPhysics();
	gnrr->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);

	return glutmain(argc, argv,640,480,"GNRR",gnrr);
}
