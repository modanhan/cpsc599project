//==============================================================================
/*
	\authors    Modan Han; Camilo Talero
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------

#include "MyPhysicsObject.h"

#include <cmath>


using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
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


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a label to display the rates [Hz] at which the simulation is running
cLabel* labelRates;

// a small sphere (cursor) representing the haptic device 
cShapeSphere* cursor;




cShapeBox* ground;
MyShapeMachingObject* sphere0;
vector<MyShapeMachingObject*> sphereShapeMatch;



cMesh* shapeMatchTriangle;
cVector3d op0, op1, op2, f0, f1, f2, v0, v1, v2;
double m = 1, k = 100, b = 10;

cShapeLine* l0;
cShapeLine* l1;
cShapeLine* l2;



// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = false;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
	TEMPLATE:    application.cpp

	Description of your application.
*/
//==============================================================================

int main(int argc, char* argv[])
{
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------

	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "CHAI3D" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	cout << "[f] - Enable/Disable full screen mode" << endl;
	cout << "[m] - Enable/Disable vertical mirroring" << endl;
	cout << "[q] - Exit application" << endl;
	cout << endl << endl;


	//--------------------------------------------------------------------------
	// OPENGL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
	// initialize GLEW library
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif


	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setBlack();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(0.25, 0.0, 0.0),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),    // look at position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

// set the near and far clipping planes of the camera
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.01);
	camera->setStereoFocalLength(0.5);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a directional light source
	light = new cDirectionalLight(world);

	// insert light source inside world
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// define direction of light beam
	light->setDir(-0.5, 0.5, -1);

	// create a sphere (cursor) to represent the haptic device
	cursor = new cShapeSphere(0.01);
	cursor->m_material->setOrangeCoral();

	// insert cursor inside world
	world->addChild(cursor);


	// ================================================================================================
	// shape matching with basic sphere particles
	// ================================================================================================

	sphere0 = new MyShapeMachingObject(new cShapeSphere(0.01), cVector3d(0, 0, 0.05));
	sphere0->m_mass = 50;
	sphere0->m_spring = 1000;
	sphere0->m_damper = 50;
	sphere0->m_object->setEnabled(0);
	world->addChild(sphere0->m_object);

	ground = new cShapeBox(1, 1, 0.01);
	ground->setLocalPos(cVector3d(0, 0, -0.005));
	ground->setEnabled(0);
	world->addChild(ground);

	for (int i = 0; i < 0; i++) {
		cVector3d p(rand() - RAND_MAX / 2, rand() - RAND_MAX / 2, rand() - RAND_MAX / 2);
		p.normalize();
		p *= 0.03;
		auto s = new MyShapeMachingObject(new cShapeSphere(0.01), p);
		s->m_mass = 1.0;
		s->m_spring = 75;
		s->m_damper = 5;
		sphereShapeMatch.push_back(s);
		world->addChild(sphereShapeMatch.back()->m_object);
	}

	// ================================================================================================
	// shape matching with 1 triangle
	// ================================================================================================

	shapeMatchTriangle = new cMesh();
	op0 = cVector3d(0.02, -0.02, 0);
	op1 = cVector3d(0.02, 0.03, 0);
	op2 = cVector3d(-0.03, 0, 0.02);
	shapeMatchTriangle->newVertex(op0);
	shapeMatchTriangle->newVertex(op1);
	shapeMatchTriangle->newVertex(op2);
	shapeMatchTriangle->setVertexColor(cColorf(1, 1, 1, 1));
	shapeMatchTriangle->newTriangle(0, 1, 2);
	world->addChild(shapeMatchTriangle);

	l0 = new cShapeLine();
	l1 = new cShapeLine();
	l2 = new cShapeLine();
	world->addChild(l0);
	world->addChild(l1);
	world->addChild(l2);
















	//--------------------------------------------------------------------------
	// HAPTIC DEVICE
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);

	// open a connection to haptic device
	hapticDevice->open();

	// calibrate device (if necessary)
	hapticDevice->calibrate();

	// retrieve information about the current haptic device
	cHapticDeviceInfo info = hapticDevice->getSpecifications();

	// display a reference frame if haptic device supports orientations
	if (info.m_sensedRotation == true)
	{
		// display reference frame
		cursor->setShowFrame(true);

		// set the size of the reference frame
		cursor->setFrameSize(0.05);
	}

	// if the device has a gripper, enable the gripper to simulate a user switch
	hapticDevice->setEnableGripperUserSwitch(true);


	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	cFontPtr font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rates of the simulation
	labelRates = new cLabel(font);
	labelRates->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(labelRates);


	//--------------------------------------------------------------------------
	// START SIMULATION
	//--------------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// setup callback when application exits
	atexit(close);


	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);

		// render graphics
		updateGraphics();

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		freqCounterGraphics.signal(1);
	}

	// close window
	glfwDestroyWindow(window);

	// terminate GLFW library
	glfwTerminate();

	// exit
	return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if (a_action != GLFW_PRESS)
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// option - toggle fullscreen
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

	// option - toggle vertical mirroring
	else if (a_key == GLFW_KEY_M)
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}
}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	hapticDevice->close();

	// delete resources
	delete hapticsThread;
	delete world;
	delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// wait until all GL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	cPrecisionClock clock;

	// main haptic simulation loop
	while (simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////
		// READ HAPTIC DEVICE
		/////////////////////////////////////////////////////////////////////

		// read position 
		cVector3d position;
		hapticDevice->getPosition(position);

		// read orientation 
		cMatrix3d rotation;
		hapticDevice->getRotation(rotation);

		// read user-switch status (button 0)
		bool button = false;
		hapticDevice->getUserSwitch(0, button);

		/////////////////////////////////////////////////////////////////////
		// UPDATE 3D CURSOR MODEL
		/////////////////////////////////////////////////////////////////////

		// update position and orienation of cursor
		cursor->setLocalPos(position);
		cursor->setLocalRot(rotation);

		/////////////////////////////////////////////////////////////////////
		// COMPUTE FORCES
		/////////////////////////////////////////////////////////////////////

		cVector3d force(0, 0, 0);
		cVector3d torque(0, 0, 0);
		double gripperForce = 0.0;

		sphere0->Update(0.001, position, force);
		for (auto& a : sphereShapeMatch) {
			a->Update(0.001, position, force);
		}


		/////////////////////////////////////////////////////////////////////
		// haptic sphere cursor against 1 triangle stuff
		/////////////////////////////////////////////////////////////////////


		auto radius = 0.01;
		auto p0 = shapeMatchTriangle->m_vertices->getLocalPos(0);
		auto p1 = shapeMatchTriangle->m_vertices->getLocalPos(1);
		auto p2 = shapeMatchTriangle->m_vertices->getLocalPos(2);

		f0 = f1 = f2 = cVector3d(0, 0, 0);

		bool contact = 0;
		auto normal = (p1 - p0); normal.cross(p2 - p0); normal.normalize();
		auto proj = (cursor->getLocalPos() - p0);
		auto proj_dot_normal = proj.dot(normal);
		auto proj_sign = (proj_dot_normal > 0) ? 1 : -1;
		proj = proj_dot_normal * normal;

		if (!contact)
		{
			// check cursor sphere against the face
			auto proj_face = cursor->getLocalPos() - proj;
			
			// calculating barycentric 
			cVector3d barycentric;
			{ auto d = p1 - p0; d.cross(proj_face - p0); barycentric.z(d.length() * (d.z() / abs(d.z()))); }
			{ auto d = p2 - p1; d.cross(proj_face - p1); barycentric.x(d.length() * (d.z() / abs(d.z()))); }
			{ auto d = p0 - p2; d.cross(proj_face - p2); barycentric.y(d.length() * (d.z() / abs(d.z()))); }
			barycentric.normalize();
 
			// cursor sphere touching triangle face
			if (abs(proj_dot_normal) < radius && barycentric.x() >= 0 && barycentric.y() >= 0 && barycentric.z() >= 0
				&& barycentric.x() <= 1 && barycentric.y() <= 1 && barycentric.z() <= 1) {
				cursor->setLocalPos(proj_face + normal * radius * proj_dot_normal / abs(proj_dot_normal));
				contact = 1;

				// direction * barycentric coordinate (weight) * penetration depth * sign * stiffness
				f0 = -normal * barycentric.x() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
				f1 = -normal * barycentric.y() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
				f2 = -normal * barycentric.z() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;

			}
		}
		if (!contact)
		{
			// cursor sphere touching line segment p0 p1
			auto d = cursor->getLocalPos() - p0;
			auto l01 = p1 - p0;
			auto lerp_amount = d.dot(l01) / l01.dot(l01);
			auto proj = l01 * lerp_amount;

			if (lerp_amount >= 0 && lerp_amount <= 1) {
				auto o = d - proj;
				auto penetration_depth = radius - o.length();
				o.normalize();
				if (penetration_depth > 0) {
					contact = 1;
					// compute cursor position
					auto target_contact = p0 + proj;
					cursor->setLocalPos(target_contact + o * radius);
					// apply forces to the triangle
					f0 = -normal * (1 - lerp_amount) * penetration_depth * proj_sign * 1000;
					f1 = -normal * (lerp_amount) * penetration_depth * proj_sign * 1000;
				}
			}
		}
		if (!contact)
		{
			// cursor sphere touching line segment p0 p1
			auto d = cursor->getLocalPos() - p1;
			auto l01 = p2 - p1;
			auto lerp_amount = d.dot(l01) / l01.dot(l01);
			auto proj = l01 * lerp_amount;

			if (lerp_amount >= 0 && lerp_amount <= 1) {
				auto o = d - proj;
				auto penetration_depth = radius - o.length();
				o.normalize();
				if (penetration_depth > 0) {
					contact = 1;
					// compute cursor position
					auto target_contact = p1 + proj;
					cursor->setLocalPos(target_contact + o * radius);
					// apply forces to the triangle
					f1 = -normal * (1 - lerp_amount) * penetration_depth * proj_sign * 900;
					f2 = -normal * (lerp_amount)* penetration_depth * proj_sign * 900;
				}
			}
		}
		if (!contact)
		{
			// cursor sphere touching line segment p0 p1
			auto d = cursor->getLocalPos() - p2;
			auto l01 = p0 - p2;
			auto lerp_amount = d.dot(l01) / l01.dot(l01);
			auto proj = l01 * lerp_amount;

			if (lerp_amount >= 0 && lerp_amount <= 1) {
				auto o = d - proj;
				auto penetration_depth = radius - o.length();
				o.normalize();
				if (penetration_depth > 0) {
					contact = 1;
					// compute cursor position
					auto target_contact = p2 + proj;
					cursor->setLocalPos(target_contact + o * radius);
					// apply forces to the triangle
					f2 = -normal * (1 - lerp_amount) * penetration_depth * proj_sign * 900;
					f0 = -normal * (lerp_amount)* penetration_depth * proj_sign * 900;
				}
			}
		}

		if (!contact)
		{
			// cursor sphere touching corner/vertex
			auto d = cursor->getLocalPos() - p0;
			auto f = d.length() - radius;
			if (f < 0) {
				d.normalize();
				cursor->setLocalPos(p0 + d * (radius));
				contact = 1;
			}
		}
		if (!contact)
		{
			// cursor sphere touching corner/vertex
			auto d = cursor->getLocalPos() - p1;
			auto f = d.length() - radius;
			if (f < 0) {
				d.normalize();
				cursor->setLocalPos(p1 + d * (radius));
				contact = 1;
			}
		}
		if (!contact)
		{
			// cursor sphere touching corner/vertex
			auto d = cursor->getLocalPos() - p2;
			auto f = d.length() - radius;
			if (f < 0) {
				d.normalize();
				cursor->setLocalPos(p2 + d * (radius));
				contact = 1;
			}
		}

		force = (cursor->getLocalPos() - position) * 2000;


		// update euler against the shape matching triangle
		{
			double delta = 0.001;

			f0 += (op0 - p0) * k - v0 * b;
			f1 += (op1 - p1) * k - v1 * b;
			f2 += (op2 - p2) * k - v2 * b;

			v0 += f0 * delta / m;
			v1 += f1 * delta / m;
			v2 += f2 * delta / m;

			p0 += v0 * delta;
			p1 += v1 * delta;
			p2 += v2 * delta;

			shapeMatchTriangle->m_vertices->setLocalPos(0, p0);
			shapeMatchTriangle->m_vertices->setLocalPos(1, p1);
			shapeMatchTriangle->m_vertices->setLocalPos(2, p2);
		}

		// update visual springs
		{
			l0->m_pointA = p0;
			l1->m_pointA = p1;
			l2->m_pointA = p2;
			l0->m_pointB = op0;
			l1->m_pointB = op1;
			l2->m_pointB = op2;
		}

		/////////////////////////////////////////////////////////////////////
		// APPLY FORCES
		/////////////////////////////////////////////////////////////////////

		// send computed force, torque, and gripper force to haptic device
		hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

		// signal frequency counter
		freqCounterHaptics.signal(1);
	}

	// exit haptics thread
	simulationFinished = true;
}

//------------------------------------------------------------------------------
