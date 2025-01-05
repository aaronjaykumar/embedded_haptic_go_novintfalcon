//==============================================================================
/*
Software License Agreement (BSD License)
Copyright (c) 2003-2016, CHAI3D.
(www.chai3d.org)

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.

* Neither the name of CHAI3D nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\author    <http://www.chai3d.org>
\author    Francois Conti
\version   3.1.0 $Rev: 1869 $
*/
//===========================================================================

//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//---------------------------------------------------------------------------
#include "src/ch_segmentTriangleCollisionChecker.h"
#include "src/ch_GOAlgorithm.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------
// stereo Mode
/*
C_STEREO_DISABLED:            Stereo is disabled
C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W = 800;
const int WINDOW_SIZE_H = 600;

// mouse menu options (right button)
const int OPTION_FULLSCREEN = 1;
const int OPTION_WINDOWDISPLAY = 2;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a little "chai3d" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;
// radius of the tool proxy
double proxyRadius;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// our collision detector for this task
ch_segmentTriangleCollisionChecker* ch_HR2Collisions;

// the GO algorithm
ch_GOAlgorithm* ch_GOAlg;


// our object of attention - we will draw a pyramid
cMesh* object;
cMultiMesh* CubeMultiMesh;
// A global function for sticking a pyramid into the world
void createPyramid(cMesh *mesh);
void createCube(cMesh *mesh, float edge, int include_top);

// set object position and orientation in global space
void setObjectPosOr();

// status of the main simulation haptics loop
bool simulationRunning = false;

// root resource path
string resourceRoot;


// has exited haptics simulation thread
bool simulationFinished = false;



//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);


int main(int argc, char* argv[])
{
	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------

	printf("\n");
	printf("-----------------------------------\n");
	printf("CHAI 3D\n");
	printf("CH lab - Haptic Rendering part II\n");
	printf("-----------------------------------\n");
	printf("\n\n");
	printf("Keyboard Options:\n\n");
	printf("[1] - texture   (ON/OFF)\n");
	printf("[2] - wireframe (ON/OFF)\n");
	printf("[x] - exit application\n");
	printf("\n\n");

	// parse first arg to try and locate resources
	resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);

	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLUT
	glutInit(&argc, argv);

	// retrieve  resolution of computer display and position window accordingly
	int screenW = glutGet(GLUT_SCREEN_WIDTH);
	int screenH = glutGet(GLUT_SCREEN_HEIGHT);
	int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
	int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

	// initialize the OpenGL GLUT window
	glutInitWindowPosition(windowPosX, windowPosY);
	glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);

	if (stereoMode == C_STEREO_ACTIVE)
		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
	else
		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

	// create display context and initialize GLEW library
	glutCreateWindow(argv[0]);

	#ifdef GLEW_VERSION
	// initialize GLEW
	glewInit();
	#endif

	// setup GLUT options
	glutDisplayFunc(updateGraphics);
	glutKeyboardFunc(keySelect);
	glutReshapeFunc(resizeWindow);
	glutSetWindowTitle("CH lab - Haptic Rendering part II");

	// set fullscreen mode
	if (fullscreen)
	{
		glutFullScreen();
	}

	// create a mouse menu (right button)
	glutCreateMenu(menuSelect);
	glutAddMenuEntry("full screen", OPTION_FULLSCREEN);
	glutAddMenuEntry("window display", OPTION_WINDOWDISPLAY);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

		//-----------------------------------------------------------------------
		// 3D - SCENEGRAPH
		//-----------------------------------------------------------------------

		// create a new world.
		world = new cWorld();

		// set the background color of the environment
		// the color is defined by its (R,G,B) components.
		world->setBackgroundColor(0.0, 0.0, 0.0);

		// create a camera and insert it into the virtual world
		camera = new cCamera(world);
		world->addChild(camera);

		// position and oriente the camera
		camera->set(cVector3d(3.0, 0.0, 0.0),    // camera position (eye)
			cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
			cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

		// set the near and far clipping planes of the camera
		// anything in front/behind these clipping planes will not be rendered
		camera->setClippingPlanes(0.01, 10.0);

		// create a light source and attach it to the camera
		light = new cDirectionalLight(world);
		camera->addChild(light);                   // attach light to camera
		light->setEnabled(true);                   // enable light source
		light->setLocalPos(cVector3d(1.0, 0.5, 1.0));  // position the light source
		light->setDir(cVector3d(-1.0, 0.5, 1.0));  // define the direction of the light beam



		//-----------------------------------------------------------------------
		// HAPTIC DEVICES / TOOLS
		//-----------------------------------------------------------------------

		// create a haptic device handler
		handler = new cHapticDeviceHandler();

		// get access to the first available haptic device
		handler->getDevice(hapticDevice, 0);

		// retrieve information about the current haptic device
		cHapticDeviceInfo info;
		if (hapticDevice)
		{
			info = hapticDevice->getSpecifications();
		}

		// create a 3D tool and add it to the world
		tool = new cToolCursor(world);
		world->addChild(tool);

		// connect the haptic device to the tool
		tool->setHapticDevice(hapticDevice);

		// initialize tool by connecting to haptic device
		tool->start();

		// map the physical workspace of the haptic device to a larger virtual workspace.
		// Phantom Omni physical workspace radius is around 0.1m , this creates a scale factor of 
		// approx. 10 from the physical to the virtual workspace, if 1.0 is passed to the following function
		tool->setWorkspaceRadius(1.5);

		// define a radius for the tool (sphere representing the device)
		tool->setRadius(0.1);

		// show  device & the proxy.
		tool->m_hapticPoint->m_sphereGoal->setShowEnabled(true);
		tool->m_hapticPoint->m_sphereGoal->m_material->setWhite();
		tool->m_hapticPoint->m_sphereProxy->setShowEnabled(true);
		tool->m_hapticPoint->m_sphereProxy->m_material->setBlueDeepSky();
		// set the physical radius of the proxy. for performance reasons, it is
		// sometimes useful to set this value to zero when dealing with
		// complex objects.
		proxyRadius = 0.1;
		tool->m_hapticPoint->m_algorithmFingerProxy->setProxyRadius(proxyRadius);

		// inform the proxy algorithm to only check front sides of triangles
		tool->m_hapticPoint->m_algorithmFingerProxy->m_collisionSettings.m_checkForNearestCollisionOnly = true;

		// is entirely static, you can set this parameter to "false"
		tool->m_hapticPoint->m_algorithmFingerProxy->m_useDynamicProxy = false;


		// read the scale factor between the physical workspace of the haptic
		// device and the virtual workspace defined for the tool
		double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

		// define a maximum stiffness that can be handled by the current
		// haptic device. The value is scaled to take into account the
		// workspace scale factor
		double stiffnessMax = info.m_maxLinearStiffness / workspaceScaleFactor;


		// define the maximum damping factor that can be handled by the
		// current haptic device. The The value is scaled to take into account the
		// workspace scale factor
		double dampingMax = info.m_maxLinearDamping / workspaceScaleFactor;


		//-----------------------------------------------------------------------
		// COMPOSE THE VIRTUAL SCENE
		//-----------------------------------------------------------------------

		// the object in our virtual scene
		object = new cMesh();

		//Choose Cube or Pyramid
		//createPyramid(object);
		createCube(object, 1.0, 0);

		// set object position and orientation in global space
		setObjectPosOr();

		object->computeAllNormals();
		object->setShowNormals(true);
		object->setShowFrame(true);
		//object->setWireMode(true);	

		bool fileload;
		object->m_texture = cTexture2d::create();
		fileload = object->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/chrome.jpg"));
		if (!fileload)
		{
		#if defined(_MSVC)
			fileload = object->m_texture->loadFromFile("../../../bin/resources/images/chrome.jpg");
		#endif
		}
		if (!fileload)
		{
			printf("Error - Texture image failed to load correctly.\n");
			close();
			return (-1);
		}


		// and haptic properties	  
		object->setStiffness(stiffnessMax, true);
		object->m_material->setViscosity(dampingMax);
		object->setUseVertexColors(true);

		// add object to world
		CubeMultiMesh = new cMultiMesh();
	
		CubeMultiMesh->addMesh(object);
		world->addChild(CubeMultiMesh);

		


		// debug code
		_cprintf("\ntranslation: %lf %lf %lf\n", object->getLocalPos().x(), object->getLocalPos().y(), object->getLocalPos().z());
		_cprintf("\nrotation: %lf %lf %lf\n", object->getLocalRot().getCol0().x(), object->getLocalRot().getCol0().y(), object->getLocalRot().getCol0().z());
		_cprintf("rotation: %lf %lf %lf\n", object->getLocalRot().getCol1().x(), object->getLocalRot().getCol1().y(), object->getLocalRot().getCol1().z());
		_cprintf("rotation: %lf %lf %lf\n", object->getLocalRot().getCol2().x(), object->getLocalRot().getCol2().y(), object->getLocalRot().getCol2().z());

		

		//-----------------------------------------------------------------------
		// START SIMULATION
		//-----------------------------------------------------------------------

		// simulation in now running
		simulationRunning = true;

		// create a thread which starts the main haptics rendering loop
		cThread* hapticsThread = new cThread();
		hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

		// start the main graphics rendering loop
		glutMainLoop();

		// close everything
		close();

		// exit
		return (0);
	}

	//---------------------------------------------------------------------------

	void resizeWindow(int w, int h)
	{
		// update the size of the viewport
		displayW = w;
		displayH = h;
		glViewport(0, 0, displayW, displayH);
	}

	//---------------------------------------------------------------------------

	void keySelect(unsigned char key, int x, int y)
	{
		// escape key
		if ((key == 27) || (key == 'x'))
		{
			// close everything
			close();

			// exit application
			exit(0);
		}

		// option 1:
		if (key == '1')
		{
			bool useTexture = object->getUseTexture();
			object->setUseTexture(!useTexture);
		}

		// option 2:
		if (key == '2')
		{
			bool useWireMode = object->getWireMode();
			object->setWireMode(!useWireMode, true);
		}
	}


	//---------------------------------------------------------------------------

	void menuSelect(int value)
	{
		switch (value)
		{
			// enable full screen display
		case OPTION_FULLSCREEN:
			glutFullScreen();
			break;

			// reshape window to original size
		case OPTION_WINDOWDISPLAY:
			glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
			break;
		}
	}

	//---------------------------------------------------------------------------

	void close(void)
	{
		// stop the simulation
		simulationRunning = false;

		// wait for graphics and haptics loops to terminate
		while (!simulationFinished) { cSleepMs(100); }

		// close haptic device
		tool->stop();
	}

	//---------------------------------------------------------------------------

	void updateGraphics(void)
	{
		// render world
		camera->renderView(displayW, displayH);

		// Swap buffers
		glutSwapBuffers();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

		// inform the GLUT window to call updateGraphics again (next frame)
		if (simulationRunning)
		{
			glutPostRedisplay();
		}
	}

	//---------------------------------------------------------------------------

	void updateHaptics(void)
	{
		bool first_time_here = true, first_time_here_too = false;

		// main haptic simulation loop
		while (simulationRunning)
		{
			static unsigned int highlight_wait;

			cVector3d ch_feedbackForce;
			cVector3d ch_lastDevicePosition;
			static cVector3d ch_nextProxyPos;

			//// slow down the haptic loop
			//cSleepMs(10);

			// compute global reference frames for each object
			world->computeGlobalPositions(true);

			// update device ("goal") pose
			tool->updateFromDevice();
			tool->updateToolImagePosition();

			tool->computeInteractionForces();

			if (first_time_here)
			{
				ch_nextProxyPos.zero();

				// initialize the collision checker when first time here 
				ch_HR2Collisions = new ch_segmentTriangleCollisionChecker(CubeMultiMesh);
				ch_GOAlg = new ch_GOAlgorithm();

				first_time_here = false;	// never enter here again
				first_time_here_too = true;
			}


			// check for GO-goal segment collisions with our object
			cVector3d device_pos, intersectionPt;



			//---------------------------uncomment this block for triangle highlighting, without feedback force!--------------------------------------//
			// collision detection and touched primitive highlighting
			device_pos = tool->getDeviceLocalPos();
			ch_HR2Collisions->ch_checkCollisions(ch_lastDevicePosition, device_pos, intersectionPt);		
			

			ch_HR2Collisions->ch_highlightTriangles();		

			if(highlight_wait == 5000)
			{	
				// wait for some time before resetting colors,
				// otherwise the highlighting goes unnoticed
				ch_HR2Collisions->ch_unHighlightTriangles();		
				highlight_wait = 0;
			}
			highlight_wait++;
			
			//last device position required in the next iteration to form the GO-goal segment
			ch_lastDevicePosition.copyfrom(device_pos);	
			tool->m_hapticPoint->m_algorithmFingerProxy->setProxyGlobalPosition(device_pos);


			// clear the collided-triangle index list from the previous iteration
			ch_HR2Collisions->ch_clearCollidedTriangleIndex();

			
			// send forces to device
			tool->applyToDevice();

			//---------------------------uncomment this block for triangle highlighting!--------------------------------------//




			//---------------------------uncomment this block for collision detection with feedback force!--------------------------------------//
			////collision detection
			//ch_HR2Collisions->ch_checkCollisions(ch_nextProxyPos, tool->getDeviceGlobalPos(), intersectionPt);
			//		
			//ch_feedbackForce = ch_GOAlg->ch_GOComputeForces(ch_HR2Collisions, ch_nextProxyPos, tool->getDeviceGlobalPos());
			//
			//if(first_time_here_too)
			//{
			//	// only for the first iteration, let the next proxy position be equal to the device position
			//	ch_nextProxyPos.copyfrom(tool->getDeviceLocalPos());
			//	first_time_here_too = false;
			//}

			//// compensate for the radius of the proxy sphere
			//cVector3d ray_device_proxy;		
			//ch_nextProxyPos.subr(tool->getDeviceGlobalPos(), ray_device_proxy);		
			//
			//if(ray_device_proxy.lengthsq())
			//{
			//	double ray_length;
			//	
			//	ray_length = ray_device_proxy.length();
			//	ray_device_proxy.normalize();

			//	ray_length = ray_length + proxyRadius;
			//	ray_device_proxy.mul(ray_length);
			//}		
			//			
			//tool->getDeviceLocalPos().addr(ray_device_proxy, ch_nextProxyPos);		
			//// set the proxy position on the surface of the virtual object
			//tool->m_hapticPoint->m_sphereProxy->setLocalPos(ch_nextProxyPos);
			//
			//// clear the collided-triangle index list from the previous iteration
			//ch_HR2Collisions->ch_clearCollidedTriangleIndex();
			//
			//tool->setDeviceGlobalForce(ch_feedbackForce);
			//// send forces to device
			//tool->applyToDevice();
			//---------------------------uncomment this block for collision detection with feedback force!--------------------------------------//
		}

		// exit haptics thread
		simulationFinished = true;
	}




	// A global function for sticking a cube in the given mesh
	// 
	// Manually creates the 12 triangles (two per face) required to
	// model a cube
	void createCube(cMesh *mesh, float edge, int include_top) {

		// I define the cube's "radius" to be half the edge size
		float radius = edge / 2.0;
		int n;
		int cur_index = 0;
		int start_index = 0;

		// +x face
		mesh->newVertex(radius, radius, -radius);
		mesh->newVertex(radius, radius, radius);
		mesh->newVertex(radius, -radius, -radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(radius, -radius, -radius);
		mesh->newVertex(radius, radius, radius);
		mesh->newVertex(radius, -radius, radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		for (n = start_index; n<cur_index; n++) {
			
			mesh->m_vertices->setTexCoord(n, (mesh->m_vertices->getLocalPos(n).y() + radius) / (2.0 * radius), (mesh->m_vertices->getLocalPos(n).z() + radius) / (2.0 * radius));
			mesh->m_vertices->setNormal(n, 1, 0, 0);
		}

		start_index += 6;

		// -x face
		mesh->newVertex(-radius, radius, radius);
		mesh->newVertex(-radius, radius, -radius);
		mesh->newVertex(-radius, -radius, -radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(-radius, radius, radius);
		mesh->newVertex(-radius, -radius, -radius);
		mesh->newVertex(-radius, -radius, radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		for (n = start_index; n<cur_index; n++) {
			
			mesh->m_vertices->setTexCoord(n, (mesh->m_vertices->getLocalPos(n).y() + radius) / (2.0 * radius), (mesh->m_vertices->getLocalPos(n).z() + radius) / (2.0 * radius));
			mesh->m_vertices->setNormal(n, -1, 0, 0);

		}

		start_index += 6;

		// +y face
		mesh->newVertex(radius, radius, radius);
		mesh->newVertex(radius, radius, -radius);
		mesh->newVertex(-radius, radius, -radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(radius, radius, radius);
		mesh->newVertex(-radius, radius, -radius);
		mesh->newVertex(-radius, radius, radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		for (n = start_index; n<cur_index; n++) {
		
			mesh->m_vertices->setTexCoord(n, (mesh->m_vertices->getLocalPos(n).x() + radius) / (2.0 * radius), (mesh->m_vertices->getLocalPos(n).z() + radius) / (2.0 * radius));
			mesh->m_vertices->setNormal(n, 0, 1, 0);

		}

		start_index += 6;

		// -y face
		mesh->newVertex(radius, -radius, radius);
		mesh->newVertex(-radius, -radius, -radius);
		mesh->newVertex(radius, -radius, -radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(-radius, -radius, -radius);
		mesh->newVertex(radius, -radius, radius);
		mesh->newVertex(-radius, -radius, radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		for (n = start_index; n<cur_index; n++) {
		
			mesh->m_vertices->setTexCoord(n, (mesh->m_vertices->getLocalPos(n).x() + radius) / (2.0 * radius), (mesh->m_vertices->getLocalPos(n).z() + radius) / (2.0 * radius));
			mesh->m_vertices->setNormal(n, 0, -1, 0);
		}

		start_index += 6;

		// -z face
		mesh->newVertex(-radius, -radius, -radius);
		mesh->newVertex(radius, radius, -radius);
		mesh->newVertex(radius, -radius, -radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(radius, radius, -radius);
		mesh->newVertex(-radius, -radius, -radius);
		mesh->newVertex(-radius, radius, -radius);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		for (n = start_index; n<cur_index; n++) {
			
			mesh->m_vertices->setTexCoord(n, (mesh->m_vertices->getLocalPos(n).x() + radius) / (2.0 * radius), (mesh->m_vertices->getLocalPos(n).y() + radius) / (2.0 * radius));
			mesh->m_vertices->setNormal(n, 0, 0, -1);

		}

		start_index += 6;

		if (include_top) {

			// +z face
			mesh->newVertex(-radius, -radius, radius);
			mesh->newVertex(radius, -radius, radius);
			mesh->newVertex(radius, radius, radius);
			mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
			cur_index += 3;

			mesh->newVertex(-radius, -radius, radius);
			mesh->newVertex(radius, radius, radius);
			mesh->newVertex(-radius, radius, radius);
			mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
			cur_index += 3;

			for (n = start_index; n<cur_index; n++) {
				mesh->m_vertices->setTexCoord(n, (mesh->m_vertices->getLocalPos(n).x() + radius) / (2.0 * radius), (mesh->m_vertices->getLocalPos(n).y() + radius) / (2.0 * radius));
				mesh->m_vertices->setNormal(n, 0, 0, 1);
			}

			start_index += 6;
		}

		// Give a color to each vertex
		for (unsigned int i = 0; i<mesh->getNumVertices(); i++) {
						
			cColorb color;
			color.set(
				GLuint(0xff * (edge + mesh->m_vertices->getLocalPos(i).x()) / (2.0 * edge)),
				GLuint(0xff * (edge + mesh->m_vertices->getLocalPos(i).y()) / (2.0 * edge)),
				GLuint(0xff * mesh->m_vertices->getLocalPos(i).z() / 2 * edge)
				);
			mesh->m_vertices->setColor(i,color);
			
		}


		// Give him some material properties...
		cMaterial material;
		material.m_ambient.set(0.6, 0.3, 0.3, 1.0);
		material.m_diffuse.set(0.8, 0.6, 0.6, 1.0);
		material.m_specular.set(0.9, 0.0, 0.0, 1.0);
		material.setShininess(100);
		mesh->setMaterial(material);

	}



	// our object of attention - we will draw a pyramid
	void createPyramid(cMesh *mesh)
	{
		unsigned int cur_index = 0;
		unsigned int start_index = 0;

		double multiplier = 0.25;

		mesh->newVertex(multiplier*0.0, multiplier*0.0, multiplier*4.0f);
		mesh->newVertex(multiplier*2.5, multiplier*2.5, multiplier*0.0);
		mesh->newVertex(multiplier*2.5, multiplier*-2.5, multiplier*0.0);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(multiplier*0.0, multiplier*0.0, multiplier*4.0f);
		mesh->newVertex(multiplier*2.5, multiplier*-2.5, multiplier*0.0);
		mesh->newVertex(multiplier*-2.5, multiplier*-2.5, multiplier*0.0);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(multiplier*0.0, multiplier*0.0, multiplier*4.0f);
		mesh->newVertex(multiplier*-2.5, multiplier*-2.5, multiplier*0.0);
		mesh->newVertex(multiplier*-2.5, multiplier*2.5, multiplier*0.0);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(multiplier*0.0, multiplier*0.0, multiplier*4.0f);
		mesh->newVertex(multiplier*-2.5, multiplier*2.5, multiplier*0.0);
		mesh->newVertex(multiplier*2.5, multiplier*2.5, multiplier*0.0);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(multiplier*2.5, multiplier*-2.5, multiplier*0.0);
		mesh->newVertex(multiplier*2.5, multiplier*2.5, multiplier*0.0);
		mesh->newVertex(multiplier*-2.5, multiplier*-2.5, multiplier*0.0);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		mesh->newVertex(multiplier*-2.5, multiplier*-2.5, multiplier*0.0);
		mesh->newVertex(multiplier*2.5, multiplier*2.5, multiplier*0.0);
		mesh->newVertex(multiplier*-2.5, multiplier*2.5, multiplier*0.0);
		mesh->newTriangle(cur_index, cur_index + 1, cur_index + 2);
		cur_index += 3;

		// Give a color to each vertex
		for (unsigned int i = 0; i<mesh->getNumVertices(); i++) {

			
			cColorb color;
			color.set(
				GLuint(0xff * (1.0 + mesh->m_vertices->getLocalPos(i).x()) / (2.0 * 1.0)),
				GLuint(0xff * (1.0 + mesh->m_vertices->getLocalPos(i).y()) / (2.0 * 1.0)),
				GLuint(0xff * mesh->m_vertices->getLocalPos(i).z() / 2 * 1.0)
				);
			mesh->m_vertices->setColor(i,color);
			//nextVertex->setColor(color);
		}
	}


	void setObjectPosOr()
	{

		// position of the object in the world	
		cVector3d object_pos(0.0, 0.5, 0.0);

		// rotate object 
		cVector3d z_axis(0.0, 0.0, 1.0);

		// angle is in radians
		double z_rotation_angle = C_PI / 4.0;

		cVector3d y_axis(0.0, 1.0, 0.0);

		// angle is in radians
		double y_rotation_angle = C_PI / 4.0;

		cVector3d x_axis(1.0, 0.0, 0.0);

		// angle is in radians
		double x_rotation_angle = C_PI / 4.0;

		// set the rotation of the object
		object->rotateAboutGlobalAxisRad(z_axis, z_rotation_angle);
		object->rotateAboutGlobalAxisRad(y_axis, y_rotation_angle);
		object->rotateAboutGlobalAxisRad(x_axis, x_rotation_angle);

		// set the position of the object 22
		object->setLocalPos(object_pos);
	}