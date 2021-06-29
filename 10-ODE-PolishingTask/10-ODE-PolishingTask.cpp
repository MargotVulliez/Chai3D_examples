//===========================================================================
/*

    \author    Margot

*/
//===========================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#include "CODE.h"
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


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelFeedback;

// stiffness of virtual spring
double linGain = 0.2;
double angGain = 0.03;
double linG;
double angG;
double linStiffness = 800;
double angStiffness = 30;

// Computed force and torque feedbcak
cVector3d force, torque;

// properties of haptic device and workspace
double workspaceScaleFactor;
double RotScaleFactor;
double maxStiffness;
double maxRotStiffness;
double maxLinearForce;
double maxRotTorque;

// Virtual workspace force in device coordinates
cVector3d VirtualWSTorque;
// Avatar virtual workspace force in device coordinates
cVector3d VirtualAvatarWSTorque;
// Virtual workspace stiffness factor
double KVirtual=1.0;

// rotational drift parameters
double KdR = 0.1; // rotational drift factor
double KvR = 0.1; // rotational drift controller gain
double ThetaMax = 20.0; // device max angular motion range
double ThetaE = 85.0; // virtual environment max angular motion range

// drift velocity in device coordinates
cVector3d RotDriftVel;
// avatar velocity in device coordinates
cVector3d RotAvatarVel;
// drift force in device coordinates
cVector3d RotDriftForce;
//avatar rotation in device coordinates
cVector3d avatarRotVect;
cMatrix3d avatarRot;
cMatrix3d avatarGlobalRot;

// orientation of the avatar workspace (Theta_w)
cVector3d centerRot;
double angleCenter;
cMatrix3d VirtualWSRot;
cVector3d posAvatar;
cMatrix3d rotAvatar;

// initial orientation of the haptic device (Theta_d0)
cVector3d deviceRotIni; 
// orientation of the haptic device (Theta_d)
cVector3d deviceRot;
cMatrix3d deviceRotMat;


// cross rotational vector (Theta_d x Theta_d0) 
cVector3d RotCrossVector;
// rotational velocity of the haptic device
cVector3d RotDeviceVel;
cVector3d TiltDeviceVel;

//State machine of the haptic loop
int stateHaptic = 0;

// Rotation
double angle;
double angleTheta;
double angleAvatar;
cVector3d axis;
cVector3d axisTheta;
cVector3d axisThetaPast;




//---------------------------------------------------------------------------
// ODE MODULE VARIABLES
//---------------------------------------------------------------------------

// ODE world
cODEWorld* ODEWorld;

// ODE objects
cODEGenericBody* ODEBlade;
cODEGenericBody* ODETool;

cODEGenericBody* ODEGPlane0;
cODEGenericBody* ODEGPlane1;
cODEGenericBody* ODEGPlane2;
cODEGenericBody* ODEGPlane3;
cODEGenericBody* ODEGPlane4;
cODEGenericBody* ODEGPlane5;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

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

// root resource path
string resourceRoot;


//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())



//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

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


//===========================================================================
/*
    DEMO:    10-ODE-PolishingTask.cpp

    This example simulates the haptic interaction of a grinder with a blade surface during a polishing operation. The simulation runs the ODE framework to evaluate the dynamics and the finger-proxy to compute the interaction forces.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 10-ODE-PolishingTask" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[h] - Display help menu" << endl;
    cout << "[1] - Enable gravity" << endl;
    cout << "[2] - Disable gravity" << endl << endl;
    cout << "[3] - decrease linear haptic gain" << endl;
    cout << "[4] - increase linear haptic gain" << endl;
    cout << "[5] - decrease angular haptic gain" << endl;
    cout << "[6] - increase angular haptic gain" << endl  << endl;
    cout << "[7] - decrease linear stiffness" << endl;
    cout << "[8] - increase linear stiffness" << endl;
    cout << "[9] - decrease angular stiffness" << endl;
    cout << "[0] - increase angular stiffness" << endl << endl;
    cout << "[q] - Exit application\n" << endl;
    cout << endl << endl;


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

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

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------
    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(2.5, 0.0, 0.3),    // camera position (eye)
                cVector3d(0.0, 0.0,-0.5),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos( 0, 0, 1.2);

    // define the direction of the light beam
    light->setDir(0,0,-1.0);

    // set uniform concentration level of light 
    light->setSpotExponent(0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(45);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();



    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);


    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.3);

    // define a radius for the tool
    tool->setRadius(0.0);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, true);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the froce and torque feedback
    labelFeedback = new cLabel(font);
    labelFeedback->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelFeedback);


    //////////////////////////////////////////////////////////////////////////
    // ODE WORLD
    //////////////////////////////////////////////////////////////////////////

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    maxRotStiffness = hapticDeviceInfo.m_maxAngularStiffness/ workspaceScaleFactor;

    maxLinearForce = hapticDeviceInfo.m_maxLinearForce;
    maxRotTorque = hapticDeviceInfo.m_maxAngularTorque;

    // clamp the force output gain to the max device stiffness
    linGain = cMin(linGain, maxStiffness / linStiffness);

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));

    // define damping properties
    ODEWorld->setAngularDamping(0.00002);
    ODEWorld->setLinearDamping(0.00002);


    //////////////////////////////////////////////////////////////////////////
    // PARAMETERS INITIALIZATION
    //////////////////////////////////////////////////////////////////////////


    // Set device initial orientation Theta_d0 (in local coordinates)
    deviceRotIni.set(0.0,0.0,1.0);
    
    // Initialize avatar orientation center
    centerRot.set(0.0,0.0,1.0);
    VirtualWSRot.identity();

    axisThetaPast.set(1.0,0.0,0.0);
    avatarRot.identity();

    // update position and orientation of tool
    tool->updateFromDevice();
    posAvatar = tool->m_hapticPoint->getGlobalPosProxy();
    rotAvatar = tool->getDeviceGlobalRot();









    //////////////////////////////////////////////////////////////////////////
    // BLADE
    //////////////////////////////////////////////////////////////////////////

    // create a new ODE object representing the blade
    ODEBlade = new cODEGenericBody(ODEWorld);

    // create a virtual mesh  that will be used for the geometry representation of the dynamic body
    cMultiMesh* blade = new cMultiMesh();

    // load model
    bool fileload;
    fileload = blade->loadFromFile(RESOURCE_PATH("../resources/models/Polishing/blade.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = blade->loadFromFile("../../../bin/resources/models/Polishing/blade.3ds");
        #endif
    }
    
    // scale object
    blade->scale(0.012);
   
    // create collision detector
    blade->createAABBCollisionDetector(0.0);

    // assign haptic properties
    cMaterial matBlade;
    matBlade.setStiffness(0.3 * maxStiffness);
    matBlade.setHapticTriangleSides(true, false);
    blade->setMaterial(matBlade);
   
    // add mesh to ODE object
    ODEBlade->setImageModel(blade);

    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    ODEBlade->createDynamicMesh(true);

    // position and orient model
    ODEBlade->setLocalPos( 0.0, 0.0,-0.5);
    ODEBlade->rotateAboutGlobalAxisDeg(cVector3d(1,0,0), 90);


    //////////////////////////////////////////////////////////////////////////
    // TOOL
    //////////////////////////////////////////////////////////////////////////

    // create a virtual tool
   ODETool = new cODEGenericBody(ODEWorld);
    cMultiMesh* imgTool = new cMultiMesh();

    fileload = imgTool->loadFromFile(RESOURCE_PATH("../resources/models/Polishing/ToolPoli2.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = imgTool->loadFromFile("../../../bin/resources/models/Polishing/ToolPoli2.3ds");
        #endif
    }
    imgTool->scale(0.065);

    // define material properties
    cMaterial matTool;
    matTool.setGrayLevel(0.3);
    matTool.setRedIndian();
    matTool.m_specular.set(0.0, 0.0, 0.0);
    matTool.setDynamicFriction(0.2);
    matTool.setStaticFriction(0.2);
    imgTool->setMaterial(matTool, true);
    imgTool->setHapticEnabled(false);

    // add mesh to ODE object
    ODETool->setImageModel(imgTool);
    ODETool->createDynamicMesh(false);

    // define mass properties
    ODETool->setMass(0.01);
    dBodySetAngularDamping(ODETool->m_ode_body, 0.06);
    dBodySetLinearDamping(ODETool->m_ode_body, 0.06);


//   // create a virtual tool
//    ODETool = new cODEGenericBody(ODEWorld);
//    cMesh* objectTool = new cMesh();
//    cVector3d offset(0.0, 0.0, -0.15);
//    cCreateCylinder(objectTool, 0.3, 0.03,12, 1,1, true, true, offset);
//
//    // define some material properties for each cube
//    cMaterial matTool;
//    matTool.m_ambient.set(0.4f, 0.4f, 0.4f);
//    matTool.m_diffuse.set(0.8f, 0.8f, 0.8f);
//    matTool.m_specular.set(1.0f, 1.0f, 1.0f);
//    matTool.setDynamicFriction(0.2);
//    matTool.setStaticFriction(0.2);
//    objectTool->setMaterial(matTool);
//
//    // add mesh to ODE object
//    ODETool->setImageModel(objectTool);
//
//    ODETool->createDynamicCylinder(0.1, 0.3);
//
//    // define some mass properties for the tool
//    ODETool->setMass(0.01);
//    dBodySetAngularDamping(ODETool->m_ode_body, 0.06);
//    dBodySetLinearDamping(ODETool->m_ode_body, 0.06);

    //////////////////////////////////////////////////////////////////////////
    // 6 ODE INVISIBLE WALLS
    //////////////////////////////////////////////////////////////////////////

    // we create 6 static walls to contains the 3 cubes within a limited workspace
//    ODEGPlane0 = new cODEGenericBody(ODEWorld);
//    ODEGPlane1 = new cODEGenericBody(ODEWorld);
//    ODEGPlane2 = new cODEGenericBody(ODEWorld);
//    ODEGPlane3 = new cODEGenericBody(ODEWorld);
//    ODEGPlane4 = new cODEGenericBody(ODEWorld);
//    ODEGPlane5 = new cODEGenericBody(ODEWorld);
//
//    w = 1.0;
//    ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0,  2.0 * w), cVector3d(0.0, 0.0 ,-1.0));
//    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -w), cVector3d(0.0, 0.0 , 1.0));
//    ODEGPlane2->createStaticPlane(cVector3d(0.0,  w, 0.0), cVector3d(0.0,-1.0, 0.0));
//    ODEGPlane3->createStaticPlane(cVector3d(0.0, -w, 0.0), cVector3d(0.0, 1.0, 0.0));
//    ODEGPlane4->createStaticPlane(cVector3d( w, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
//    ODEGPlane5->createStaticPlane(cVector3d(-0.8 * w, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));



    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

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

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // help menu:
    else if (a_key == GLFW_KEY_H)
    {
        cout << "Keyboard Options:" << endl << endl;
        cout << "[h] - Display help menu" << endl;
        cout << "[1] - Enable gravity" << endl;
        cout << "[2] - Disable gravity" << endl << endl;
        cout << "[3] - decrease linear haptic gain" << endl;
        cout << "[4] - increase linear haptic gain" << endl;
        cout << "[5] - decrease angular haptic gain" << endl;
        cout << "[6] - increase angular haptic gain" << endl  << endl;
        cout << "[7] - decrease linear stiffness" << endl;
        cout << "[8] - increase linear stiffness" << endl;
        cout << "[9] - decrease angular stiffness" << endl;
        cout << "[0] - increase angular stiffness" << endl << endl;
        cout << "[q] - Exit application\n" << endl;
        cout << endl << endl;
    }

    // option - enable gravity:
    else if (a_key == GLFW_KEY_1)
    {
        // enable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
        printf("gravity ON:\n");
    }

    // option - disable gravity:
    else if (a_key == GLFW_KEY_2)
    {
        // disable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
        printf("gravity OFF:\n");
    }

    // option - decrease linear haptic gain
    else if (a_key == GLFW_KEY_3)
    {
        linGain = linGain - 0.05;
        if (linGain < 0)
            linGain = 0;
        printf("linear haptic gain:  %f\n", linGain);
    }

    // option - increase linear haptic gain
    else if (a_key == GLFW_KEY_4)
    {
        linGain = linGain + 0.05;
        printf("linear haptic gain:  %f\n", linGain);
    }

    // option - decrease angular haptic gain
    else if (a_key == GLFW_KEY_5)
    {
        angGain = angGain - 0.005;
        if (angGain < 0)
            angGain = 0;
        printf("angular haptic gain:  %f\n", angGain);
    }

    // option - increase angular haptic gain
    else if (a_key == GLFW_KEY_6)
    {
        angGain = angGain + 0.005;
        printf("angular haptic gain:  %f\n", angGain);
    }

    // option - decrease linear stiffness
    else if (a_key == GLFW_KEY_7)
    {
        linStiffness = linStiffness - 50;
        if (linStiffness < 0)
            linStiffness = 0;
        printf("linear stiffness:  %f\n", linStiffness);
    }

    // option - increase linear stiffness
    else if (a_key == GLFW_KEY_8)
    {
        linStiffness = linStiffness + 50;
        printf("linear stiffness:  %f\n", linStiffness);
    }

    // option - decrease angular stiffness
    else if (a_key == GLFW_KEY_9)
    {
        angStiffness = angStiffness - 1;
        if (angStiffness < 0)
            angStiffness = 0;
        printf("angular stiffness:  %f\n", angStiffness);
    }

    // option - increase angular stiffness
    else if (a_key == GLFW_KEY_0)
    {
        angStiffness = angStiffness + 1;
        printf("angular stiffness:  %f\n", angStiffness);
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

//---------------------------------------------------------------------------

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

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelFeedback->setText(RotDriftForce.str(3) + "Nm" + TiltDeviceVel.str(3) + "rad/w" +  centerRot.str(3) + "NU" + avatarRotVect.str(3) + "NU" + cStr(angleTheta*180/3.14,5) + "deg");

    // update position of label
    labelFeedback->setLocalPos((int)(0.5 * (width - labelFeedback->getWidth())), 15);


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
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // start haptic device
    hapticDevice->open();

    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);

    cMatrix3d prevRotTool;
    prevRotTool.identity();

    // main haptic simulation loop
    while(simulationRunning)
    {
        // update frequency counter
        freqCounterHaptics.signal(1);

        // retrieve simulation time and compute next interval
        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = 0.00025;//cClamp(time, 0.00001, 0.0002);
        
        // reset clock
        simClock.reset();
        simClock.start();

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

	/////////////////////////////////////////////////////////////////////
        // COMPUTE ODE INTERACTION FORCES
        /////////////////////////////////////////////////////////////////////

        // read position of tool
        cVector3d posTool = ODETool->getGlobalPos();
        cMatrix3d rotTool = ODETool->getGlobalRot();

        // compute position and angular error between tool and haptic device
        cVector3d deltaPos = (posAvatar - posTool);
        cMatrix3d deltaRot = cMul(cTranspose(rotTool), rotAvatar);
        deltaRot.toAxisAngle(axis, angle);
        
        // compute force and torque to apply to tool
        force = linStiffness * deltaPos;
        torque = cMul((angStiffness * angle), axis);
        rotTool.mul(torque);  

        // compute force and torque to apply to haptic device
        force = -linG * force;
        torque = -angG * torque;

       // add force contribution from ODE model
	tool->addDeviceGlobalForce(force);
	tool->addDeviceGlobalTorque(torque);

        if (linG < linGain)
        {
            linG = linG + 0.1 * time * linGain;
        }
        else
        {
            linG = linGain;
        }

        if (angG < angGain)
        {
            angG = angG + 0.1 * time * angGain;
        }
        else
        {
            angG = angGain;
        }


	/////////////////////////////////////////////////////////////////////
        // WORKSPACE DRIFT CONTROL
        /////////////////////////////////////////////////////////////////////
	
	// get orientation of the device theta_d (in local coordinates)
 	hapticDevice->getRotation(deviceRotMat);
	deviceRot = deviceRotMat * deviceRotIni;

	// get rotational velocity of the device wd (in local coordinates)
	hapticDevice->getAngularVelocity(RotDeviceVel);
	// get tilt velocity of the device 
	TiltDeviceVel=RotDeviceVel - deviceRot.dot(RotDeviceVel)*deviceRot;

	// cross rotational vector (Theta_d x Theta_d0)
	deviceRot.crossr(deviceRotIni,RotCrossVector);
	// dot product (Theta_d . Theta_d0)
	double RotDot=deviceRot.dot(deviceRotIni);

        // calculation of the rotational drift velocity
	RotDriftVel = KdR*TiltDeviceVel.length()*RotCrossVector/(sin(ThetaMax*3.14/180));


	/////////////////////////////////////////////////////////////////////
        // SCALING FACTOR
        /////////////////////////////////////////////////////////////////////

	// Modification of the rotational scaling factor according to the orientation
	RotScaleFactor = (1 + RotCrossVector.length()*(ThetaE/ThetaMax-1)/(sin(ThetaMax*3.14/180)));


        /////////////////////////////////////////////////////////////////////
        // DRIFT TORQUE COMPUTATION
        /////////////////////////////////////////////////////////////////////

	// set avatar rotational velocity in device coordinates
	RotAvatarVel = RotScaleFactor * (RotDeviceVel-RotDriftVel);
	tool->setDeviceLocalAngVel(RotAvatarVel);
	
	
	// Virtual workspace orientation
	angleCenter = RotScaleFactor*RotDriftVel.length()/4000.0; //frequency of the Omega.7

	// Rotation axis in the virtual workspace local coordinates
	axisTheta = VirtualWSRot*RotCrossVector;
	if (axisTheta.length()!=0.0)
	{
		axisTheta.normalize();
	}
	else
	{
		axisTheta=axisThetaPast;
	}
	axisThetaPast=axisTheta;

	avatarRot.setAxisAngleRotationRad(-axisTheta, angleCenter);
	// Update new virtual workspace rotation matrix and z_axis
	VirtualWSRot=VirtualWSRot*avatarRot;
	centerRot = avatarRot * centerRot;

	// set avatar orientation in device coordinates
	angleTheta = atan2(RotCrossVector.length(),RotDot);
	
	avatarRot.setAxisAngleRotationRad(-axisTheta, RotScaleFactor*angleTheta);
	// Update new avatar orientation axis and rotation matrix
	avatarRotVect = avatarRot * centerRot;
	avatarRot=VirtualWSRot*avatarRot;

	tool->setDeviceLocalRot(avatarRot);
	avatarGlobalRot = tool->getDeviceGlobalRot();

	// drift of the device
	RotDriftForce = KvR*maxRotTorque*(RotDriftVel-TiltDeviceVel);
	//RotDriftForce = KvR*maxRotTorque*(RotDriftVel);	
	tool->addDeviceLocalTorque(RotDriftForce);



      	/////////////////////////////////////////////////////////////////////
        // ODE TOOL FORCES
        /////////////////////////////////////////////////////////////////////

        // update new position and orientation of tool after the rotational drift
        posAvatar = tool->m_hapticPoint->getGlobalPosProxy();
	rotAvatar = avatarGlobalRot;

        // compute position and angular error between tool and haptic device
        deltaPos = (posAvatar - posTool);
        deltaRot = cMul(cTranspose(rotTool), rotAvatar);
        deltaRot.toAxisAngle(axis, angle);
        
        // compute force and torque to apply to tool
        force = linStiffness * deltaPos;
        torque = cMul((angStiffness * angle), axis);
        rotTool.mul(torque);	
        
        // Apply force to the ODE tool
        ODETool->addExternalForce(force);	
        ODETool->addExternalTorque(torque);


	/////////////////////////////////////////////////////////////////////
        // CHECK WORKSPACE LIMITS
        /////////////////////////////////////////////////////////////////////


	if (angleTheta>=(ThetaMax*3.14/180))
	{
		VirtualWSTorque = KVirtual*(angleTheta-(ThetaMax*3.14/180))*maxRotStiffness*RotCrossVector;

	}

	// Set the virtual workspace force to the device
	tool->addDeviceLocalTorque(VirtualWSTorque);


	/////////////////////////////////////////////////////////////////////
        // CHECK VIRTUAL WORKSPACE LIMITS
        /////////////////////////////////////////////////////////////////////

	avatarRot.toAxisAngle(axis, angleAvatar);

	if (angleAvatar>=(ThetaE*3.14/180))
	{
		VirtualAvatarWSTorque = KVirtual*(angleAvatar-(ThetaE*3.14/180))*maxRotStiffness*RotCrossVector;
	}

	// Set the virtual workspace force to the device
	tool->addDeviceLocalTorque(VirtualAvatarWSTorque);

	/////////////////////////////////////////////////////////////////////
        // APPLY FORCE /TORQUE AND UPDATE SIMULATION
        /////////////////////////////////////////////////////////////////////

        // send forces to device
        tool->applyToDevice();

        // update simulation
        ODEWorld->updateDynamics(nextSimInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

