#ifndef raaRobotController
#define raaRobotController

#include <iostream>
#include <string>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgUtil/UpdateVisitor>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Material>

#include <osg/Light>
#include <osg/LightSource> //leaf node for defining a light in the scene
#include <osg/StateAttribute> //base class for state attributes
#include <osg/Geometry> 
#include <osg/Point> //encapsulates the opengl points smoothing & size state

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgUtil/PolytopeIntersector>
#include <osg/Camera> //a subclass of transform which represents encapsulates the settings of a camera
#include <osg/NodeCallback>
#include <osg/Group>
#include <osg/MatrixTransform> //a subclass of Transform which represents a 4x4 transformation of its children from local coordinates into the Transform's parent coordinates
#include <iostream>
#include <osg/Notify>

#include <osgFX/Outline> //used to produce the outline effect


#include "findNodeVisitor.h"
#include "Utilities_code.h"
#include "Recorder.h"

//declare globals extern to prevent duplicate symbols
extern osg::Node *g_pModel ;
extern osg::ref_ptr<osg::Node> _selectedNode;
extern osg_Recorder * RecordPlayback;

//declaration of functions
int main(int argc, char** argv);
void init();

//class declarations
class PickHandler : public osgGA::GUIEventHandler
{ //the class used for the selection of the nodes
public:
	PickHandler();
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:

	float _mX, _mY;
	//to store x & y positions for key press & moving events
	osg::Node *child;

	//boolean for the picking operation
	bool pick(const double x, const double y, osgViewer::Viewer* viewer);
};
//class to encapsulate the data so it is available to both the event handler and update callback
class robotInputDeviceStateType
{
public:
	robotInputDeviceStateType::robotInputDeviceStateType();
	bool moveRequest;
	float direction;
	float elevation;
	float uaElevation;
};

//callback for Robot Body by defining which axis of motion goes with what body part
class updateRobotPosCallback : public osg::NodeCallback
{
public:
	updateRobotPosCallback::updateRobotPosCallback(robotInputDeviceStateType* robotIDevState, std::string partName);
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
protected:
	robotInputDeviceStateType* robotInputDeviceState;
	std::string bodyPartName;

};


//keyboard event handler
class myKeyboardEventHandler : public osgGA::GUIEventHandler
{
public:
	myKeyboardEventHandler(robotInputDeviceStateType* tids);
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&);

protected:
	robotInputDeviceStateType* robotInputDeviceState;
};
#endif
