cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(include)
include_directories(include/OpenGL)
include_directories(include/gnrr)

find_package(Bullet REQUIRED)
include_directories(/usr/local/include/bullet)

OPTION(USE_GLUT "Use Glut"	ON)

FIND_PACKAGE(OpenGL)
IF (OPENGL_FOUND)
	MESSAGE("OPENGL FOUND")
	MESSAGE(${OPENGL_LIBRARIES})
ELSE (OPENGL_FOUND)
	MESSAGE("OPENGL NOT FOUND")
	SET(OPENGL_gl_LIBRARY opengl32)
	SET(OPENGL_glu_LIBRARY glu32)
ENDIF (OPENGL_FOUND)

FIND_PACKAGE(GLU)

IF (USE_GLUT)
	FIND_PACKAGE(GLUT)
	IF (GLUT_FOUND)
		MESSAGE("GLUT FOUND")
		MESSAGE(${GLUT_glut_LIBRARY})
	ENDIF (GLUT_FOUND)

	IF(NOT WIN32)	
		# This is added for linux. This should always work if everything is installed and working fine.
		INCLUDE_DIRECTORIES(/usr/include /usr/local/include) 
	ENDIF()
ENDIF(USE_GLUT)

rosbuild_add_library(OpenGLSupport
		src/OpenGL/GLDebugFont.cpp
		src/OpenGL/GL_DialogDynamicsWorld.cpp
		src/OpenGL/GL_DialogWindow.cpp
		src/OpenGL/GL_ShapeDrawer.cpp
		src/OpenGL/GL_Simplex1to4.cpp
		src/OpenGL/GLDebugDrawer.cpp
		
		src/OpenGL/RenderTexture.cpp
		src/OpenGL/DemoApplication.cpp
		
		src/OpenGL/GlutDemoApplication.cpp
		src/OpenGL/GlutStuff.cpp

		src/OpenGL/stb_image.cpp
)

target_link_libraries(OpenGLSupport ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} bullet)

rosbuild_add_executable(gnrr_test src/gnrr/GraspTest.cpp)
target_link_libraries(gnrr_test OpenGLSupport  BulletWorldImporter BulletDynamics  BulletCollision LinearMath  BulletFileLoader  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

rosbuild_add_executable(gnrr src/gnrr/gnrr.cpp)
target_link_libraries(gnrr OpenGLSupport  BulletWorldImporter BulletDynamics  BulletCollision LinearMath  BulletFileLoader  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

