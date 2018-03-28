//==============================================================================
/*
	\author    Modan
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------

#include "loader.hpp"
#include "MyPhysicsObject.h"

#include <cmath>

#define uint unsigned int

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


auto radius = 0.01;
struct MatchingShape
{
	cMultiMesh* object;
	cMesh* mesh;
	vector<cVector3d> originalPoints = vector<cVector3d>(3);
	vector<cVector3d> forces = vector<cVector3d>(3);
	vector<cVector3d> velocities = vector<cVector3d>(3);

	double m = 1, k = 100, b = 10;

	MatchingShape(string);

	void update(cVector3d &force, cVector3d &position);
	bool updateTriangle(cVector3d &force, cVector3d &position, unsigned int);
	bool updateLine(uint indexP0, uint indexP1, cVector3d &position);
	bool updatePoint(uint index, cVector3d &position);
	void movePoint(uint index);
};

MatchingShape::MatchingShape(string fileName) {
	object = new cMultiMesh();

	// load geometry from file and compute additional properties
	object->loadFromFile(fileName);
	object->createAABBCollisionDetector(0.1);
	object->computeBTN();

	// obtain the first (and only) mesh from the object
	//cMesh* mesh = object->getMesh(0);
	mesh = new cMesh();

	vector<cVector3d> vertices;
	vector<cVector3d> normals;
	vector<cVector3d> uvs;
	vector<unsigned int> indices;
	
	loadOBJ("Meshes/monkey.obj", &vertices, &normals, &uvs, &indices);

	/*vertices = {cVector3d(0.1,0.1,0), cVector3d(-0.1,0.1,0.05), cVector3d(-0.1,-0.1,0.05), cVector3d(0.1,-0.1,0)};
	indices = {0,1,2, 2,3,0};*/

	cout << vertices.size() << endl;
	cout << indices.size() << endl;
	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		//cout << vertices[i] << endl;
		mesh->newVertex(vertices[i], normals[i]);

		mesh->setVertexColor(cColorf(1, 1, 1, 1));
	}

	for (unsigned int i = 0; i < indices.size()/3; i++)
	{
		//cout << indices[i * 3] << ", " << indices[i * 3 + 1] << ", " << indices[i * 3 + 2] << endl;
		mesh->newTriangle(indices[i*3], indices[i*3+1], indices[i*3+2]);
	}

	/*auto op0 = cVector3d(0.02, -0.02, 0);
	auto op1 = cVector3d(0.02, 0.03, 0);
	auto op2 = cVector3d(-0.03, 0, 0.02);
	mesh->newVertex(op0);
	mesh->newVertex(op1);
	mesh->newVertex(op2);
	mesh->setVertexColor(cColorf(1, 1, 1, 1));
	mesh->newTriangle(0, 1, 2);*/

	// replace the object's material with a custom one
	mesh->m_material = cMaterial::create();
	mesh->m_material->setWhite();
	mesh->m_material->setUseHapticShading(true);
	//object->setStiffness(2000.0, true);

	//world->clearAllChildren();
	world->addChild(mesh);

	originalPoints = vector<cVector3d>(mesh->m_vertices->getNumElements());
	forces = vector<cVector3d>(mesh->m_vertices->getNumElements());
	velocities = vector<cVector3d>(mesh->m_vertices->getNumElements());

	for (unsigned int i = 0; i < mesh->m_vertices->getNumElements(); i++)
	{
		originalPoints[i] = mesh->m_vertices->getLocalPos(i);
		forces[i] = cVector3d(0,0,0);
		velocities[i] = cVector3d(0, 0, 0);
	}
}

void MatchingShape::update(cVector3d &force, cVector3d &position)
{
	//mesh = object->getMesh(0);
	for(unsigned int i=0; i<mesh->getNumTriangles(); i++){

		updateTriangle(force, position,  i);
	}

	force = (cursor->getLocalPos() - position) * 2000;

	for (unsigned int i = 0; i < mesh->m_vertices->getNumElements(); i++)
	{
		movePoint(i);
	}
}

bool MatchingShape::updateTriangle(cVector3d &force, cVector3d &position, unsigned int index) {
	
	unsigned int index0 = (mesh->m_triangles->getVertexIndex0(index));
	cVector3d p0 = mesh->m_vertices->getLocalPos(index0);

	unsigned int index1 = (mesh->m_triangles->getVertexIndex1(index));
	cVector3d p1 = mesh->m_vertices->getLocalPos(index1);

	unsigned int index2 = (mesh->m_triangles->getVertexIndex2(index));
	cVector3d p2 = mesh->m_vertices->getLocalPos(index2);

	forces[index0] = forces[index1] = forces[index2] = cVector3d(0, 0, 0);

	bool contact = 0;
	auto normal = (p1 - p0); normal.cross(p2 - p0); normal.normalize();
	auto proj = (cursor->getLocalPos() - p0);
	auto proj_dot_normal = proj.dot(normal);
	auto proj_sign = (proj_dot_normal > 0) ? 1 : -1;
	proj = proj_dot_normal * normal;
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
			forces[index0] = -normal * barycentric.x() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
			forces[index1] = -normal * barycentric.y() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
			forces[index2] = -normal * barycentric.z() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;

		}
	}

	return contact;
}

void MatchingShape::movePoint(uint index)
{
	double delta = 0.001;
	auto p = mesh->m_vertices->getLocalPos(index);

	forces[index] += (originalPoints[index] - p) * k - velocities[index] * b;

	velocities[index] += forces[index] * delta / m;

	p += velocities[index] * delta;

	mesh->m_vertices->setLocalPos(index, p);
}

struct MatchingTriangle{
	cMesh* triangle;
	vector<cVector3d> originalPoints = vector<cVector3d>(3);
	vector<cVector3d> forces = vector<cVector3d>(3);
	vector<cVector3d> velocities = vector<cVector3d>(3);
	
	double m = 1, k = 100, b = 10;

	cShapeLine* l0;
	cShapeLine* l1;
	cShapeLine* l2;

	MatchingTriangle();

	void update(cVector3d &force, cVector3d &position);
	bool updateTriangle(cVector3d &force, cVector3d &position);
	bool updateLine(uint indexP0, uint indexP1, cVector3d &position);
	bool updatePoint(uint index, cVector3d &position);
	void movePoint(uint index);
};

MatchingTriangle::MatchingTriangle()
{
	triangle = new cMesh();
	originalPoints[0] = cVector3d(0.02, -0.02, 0);
	originalPoints[1] = cVector3d(0.02, 0.03, 0);
	originalPoints[2] = cVector3d(-0.03, 0, 0.02);
	triangle->newVertex(originalPoints[0]);
	triangle->newVertex(originalPoints[1]);
	triangle->newVertex(originalPoints[2]);
	triangle->setVertexColor(cColorf(1, 1, 1, 1));
	triangle->newTriangle(0, 1, 2);
	world->addChild(triangle);

	l0 = new cShapeLine();
	l1 = new cShapeLine();
	l2 = new cShapeLine();
	world->addChild(l0);
	world->addChild(l1);
	world->addChild(l2);
}

MatchingTriangle* test_triangle;


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

MatchingShape* test_shape;
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
	camera->set(cVector3d(0.2, 0.0, 0.0),    // camera position (eye)
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
	// shape matching with 1 triangle
	// ================================================================================================


	//test_triangle = new MatchingTriangle();

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



	test_shape = new MatchingShape("Meshes/monkey.3ds");


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

void MatchingTriangle::movePoint(uint index)
{
	double delta = 0.001;
	auto p = triangle->m_vertices->getLocalPos(index);

	forces[index] += (originalPoints[index] - p) * k - velocities[index] * b;
	
	velocities[index] += forces[index] * delta / m;
	
	p += velocities[index] * delta;
	
	triangle->m_vertices->setLocalPos(index, p);
}

bool MatchingTriangle::updateLine(uint indexP0, uint indexP1, cVector3d &position)
{
	auto p0 = triangle->m_vertices->getLocalPos(indexP0);
	auto p1 = triangle->m_vertices->getLocalPos(indexP1);

	bool contact = false;
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
			auto fd = -(cursor->getLocalPos() - position); fd.normalize();
			forces[indexP0] = fd * (1 - lerp_amount) * penetration_depth  * 1000;
			forces[indexP1] = fd * (lerp_amount) * penetration_depth * 1000;
		}
	}

	return contact;
}

bool MatchingTriangle::updatePoint(uint index, cVector3d &position)
{
	bool contact = false;
	auto p0 = triangle->m_vertices->getLocalPos(index);

	auto d = cursor->getLocalPos() - p0;
	auto f = d.length() - radius;
	if (f < 0) {
		d.normalize();
		cursor->setLocalPos(p0 + d * (radius));
		contact = 1;
		auto fd = -(cursor->getLocalPos() - position); fd.normalize();
		forces[index] = -f * fd * 900;
	}

	return contact;
}

bool MatchingTriangle::updateTriangle(cVector3d &force, cVector3d &	position){
	auto p0 = triangle->m_vertices->getLocalPos(0);
	auto p1 = triangle->m_vertices->getLocalPos(1);
	auto p2 = triangle->m_vertices->getLocalPos(2);

	forces[0] = forces[1] = forces[2] = cVector3d(0, 0, 0);

	bool contact = 0;
	auto normal = (p1 - p0); normal.cross(p2 - p0); normal.normalize();
	auto proj = (cursor->getLocalPos() - p0);
	auto proj_dot_normal = proj.dot(normal);
	auto proj_sign = (proj_dot_normal > 0) ? 1 : -1;
	proj = proj_dot_normal * normal;
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
			forces[0] = -normal * barycentric.x() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
			forces[1] = -normal * barycentric.y() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;
			forces[2] = -normal * barycentric.z() * (radius - abs(proj_dot_normal)) * proj_sign * 1000;

		}
	}

	return contact;
}

void MatchingTriangle::update(cVector3d &force, cVector3d &	position){
	/////////////////////////////////////////////////////////////////////
	// haptic sphere cursor against 1 triangle stuff
	/////////////////////////////////////////////////////////////////////

	auto p0 = triangle->m_vertices->getLocalPos(0);
	auto p1 = triangle->m_vertices->getLocalPos(1);
	auto p2 = triangle->m_vertices->getLocalPos(2);

	bool contact = 0;

	contact = updateTriangle(force, position);

	if (!contact)
	{
		contact = test_triangle->updateLine(0, 1, position);
	}
	if (!contact)
	{
		contact = test_triangle->updateLine(1, 2, position);
	}
	if (!contact)
	{
		contact = test_triangle->updateLine(0, 2, position);
	}

	if (!contact)
	{
		contact = test_triangle->updatePoint(0, position);
	}
	if (!contact)
	{
		contact = test_triangle->updatePoint(1, position);
	}
	if (!contact)
	{
		contact = test_triangle->updatePoint(2, position);
	}

	force = (cursor->getLocalPos() - position) * 2000;


	// update euler against the shape matching triangle
	{
		movePoint(0);
		movePoint(1);
		movePoint(2);
	}

	// update visual springs
	{
		l0->m_pointA = p0;
		l1->m_pointA = p1;
		l2->m_pointA = p2;
		l0->m_pointB = originalPoints[0];
		l1->m_pointB = originalPoints[1];
		l2->m_pointB = originalPoints[2];
	}
}

cVector3d tool_center = cVector3d(0, 0, 0);
void move_center(cVector3d pos)
{
	if (pos.length() > 0.03)
	{
		tool_center += cNormalize(pos)*0.0001;

		cVector3d new_pos = camera->getLocalPos() + cNormalize(pos)*0.0001;
		camera->set(new_pos,							// camera position (eye)
			new_pos + (cVector3d(-1.0, 0.0, 0.0)),		// look at position (target)
			cVector3d(0.0, 0.0, 1.0));					// direction of the (up) vector

	}
}
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
		move_center(position);

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
		cursor->setLocalPos(position + tool_center);
		cursor->setLocalRot(rotation);

		/////////////////////////////////////////////////////////////////////
		// COMPUTE FORCES
		/////////////////////////////////////////////////////////////////////

		cVector3d force(0, 0, 0);
		cVector3d torque(0, 0, 0);
		double gripperForce = 0.0;

		test_shape->update(force, position + tool_center);

		//test_triangle->update(force, position + tool_center);

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
