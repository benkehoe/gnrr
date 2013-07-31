#include "GraspTest.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"

#include "btBulletDynamicsCommon.h"

int main(int argc,char** argv)
{

	

    GraspTest* graspTest = new GraspTest();
	

    graspTest->initPhysics();
	graspTest->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
	
	return glutmain(argc, argv,640,480,"Grasper Test. http://www.continuousphysics.com/Bullet/phpBB2/",graspTest);
}

