// raaRobotController.cpp : Defines the entry point for the console application.

#include "stdafx.h"
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
#include "findNodeVisitor.h"

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

#include <osg/ImageStream> //a library of image stream textures

#include <osgFX/Outline> //used to produce the outline effect

osg::Node *g_pModel = 0;
osg::ref_ptr<osg::Node> _selectedNode;


//the class used for the selection of the nodes
class PickHandler : public osgGA::GUIEventHandler { 
	public:

	PickHandler() : _mX(0.), _mY(0.) {}
	bool handle(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa) {
		osgViewer::Viewer* viewer =
			dynamic_cast<osgViewer::Viewer*>(&aa);
		if (!viewer)
			return(false);

		switch (ea.getEventType()) {
		case osgGA::GUIEventAdapter::PUSH: //recording mouse location required for key press & moving instances
		case osgGA::GUIEventAdapter::MOVE: {
			_mX = ea.getX();
			_mY = ea.getY();
			return(false);
		}
		case osgGA::GUIEventAdapter::RELEASE: {
		//conditioning the mouse to performa  pick if it hasn't moved since the last button press or move event, otherwise the trackball manipulator will take over
			if (_mX == ea.getX() && _mY == ea.getY()) {
				if (pick(ea.getXnormalized(),
					ea.getYnormalized(), viewer))
					return(true);
			}
			return(false);
		}
		default:
			return(false);
		}
		}

protected:
	float _mX, _mY;
	osg::Node *child; //to store x & y positions for key press & moving events
	bool pick(const double x, const double y, //boolean for the picking operation
		osgViewer::Viewer* viewer) {
		if (!viewer->getSceneData())
			return(false); //if there's nothing to pick, retun nothing

		double w(.05), h(.05);
		osgUtil::PolytopeIntersector* picker =
			new osgUtil::PolytopeIntersector(
				osgUtil::Intersector::PROJECTION,
				x - w, y - h, x + w, y + h);

		osgUtil::IntersectionVisitor iv(picker);
		viewer->getCamera()->accept(iv);

		if (picker->containsIntersections()) {
			const osg::NodePath& nodePath =
				picker->getFirstIntersection().nodePath;
			unsigned int idx = nodePath.size();
			while (idx--) { //loop to find the last MatrixTransform in the mode path which will be the Matrix Transform to attach the callback to
				osg::MatrixTransform* mt =
					dynamic_cast<osg::MatrixTransform*>(
						nodePath[idx]);
				if (mt == NULL)
					continue; //at this point a MatrixTransform would have be found in the nodePath
				if (mt->getName() == "BodyRotator" || mt->getName() == "UpperArmRotator" || mt->getName() == "LowerArmRotator" || mt->getName() == "HandRotator") {
				//if condition for if any of the above parts are selected to do the following
					if (child) { //if condition for if there's a child instance
						osg::Node * parent = child->getParent(0)->getParent(0); //to find the grandparent of the child
						parent->asGroup()->addChild(child); //to add the child to the new parent
						parent->asGroup()->removeChild(child->getParent(0)); //to remove the original parent
					}

					std::cout << "highlighting: " << mt->getName() << std::endl;

					osgFX::Outline* outline = new osgFX::Outline();

					outline->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
					outline->setWidth(6);
					outline->addChild(mt->getChild(1)); //to make the outline the parent of the child (the model)
					child = mt->getChild(1); //to make the child node the child
					mt->replaceChild(mt->getChild(1), outline);

					break;
				}
			}
		}
	}
};

//class to encapsulate the data so it is available to both the event handler and update callback
class robotInputDeviceStateType {
public:
	robotInputDeviceStateType::robotInputDeviceStateType() :
		moveRequest(false), direction(0.0), elevation(0.0), uaElevation(0.0) {}
	bool moveRequest;
	float direction;
	float elevation;
	float uaElevation;
};

//callback for Robot Body by defining which axis of motion goes with what body part
class updateRobotPosCallback : public osg::NodeCallback {
public:
	updateRobotPosCallback::updateRobotPosCallback(robotInputDeviceStateType* robotIDevState, std::string partName) {
		robotInputDeviceState = robotIDevState;
		bodyPartName = partName;
	}
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
		if (robotInputDeviceState->moveRequest) {
			osg::MatrixTransform * mt = dynamic_cast<osg::MatrixTransform *> (node);

			if (bodyPartName == "BodyRotator") {
				const osg::Vec3d axis(0, 0, 1);
				mt->setMatrix(mt->getMatrix() * osg::Matrix::rotate(robotInputDeviceState->direction, axis));
			}
			else if (bodyPartName == "LowerArmRotator") {
				const osg::Vec3d axis(0, 1, 0);
				mt->setMatrix(mt->getMatrix() * osg::Matrix::rotate(robotInputDeviceState->elevation, axis));
			}
			else if (bodyPartName == "UpperArmRotator") {
				const osg::Vec3d axis(0, 1, 0);
				mt->setMatrix(mt->getMatrix() * osg::Matrix::rotate(robotInputDeviceState->uaElevation, axis));
			}
			traverse(node, nv);
		}
	}
protected:
	robotInputDeviceStateType* robotInputDeviceState;
	std::string bodyPartName;

};


//keyboard event handler
class myKeyboardEventHandler : public osgGA::GUIEventHandler
{
public:
	myKeyboardEventHandler(robotInputDeviceStateType* tids)
	{
		robotInputDeviceState = tids;
	}
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&);

protected:
	robotInputDeviceStateType* robotInputDeviceState;
};

//key press commands to conrol the arm movement
bool myKeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	switch (ea.getEventType()) { //key press commands detection
		case osgGA::GUIEventAdapter::KEY_Down: {
			switch (ea.getKey()){
				case osgGA::GUIEventAdapter::KEY_Down:
					std::cout << "Upper Arm moving down" << std::endl;
					robotInputDeviceState->uaElevation = 0.005;
					robotInputDeviceState->moveRequest = true;
					return false;
					break;
				case osgGA::GUIEventAdapter::KEY_Up:
					std::cout << "Upper Arm moving up" << std::endl;
					robotInputDeviceState->uaElevation = -0.005;
					robotInputDeviceState->moveRequest = true;
					return false;
					break;
				case osgGA::GUIEventAdapter::KEY_Left:
					std::cout << "Robot rotating Anti-Clockwise" << std::endl;
					robotInputDeviceState->direction = 0.005;
					robotInputDeviceState->moveRequest = true;
					return false;
					break;
				case osgGA::GUIEventAdapter::KEY_Right:
					std::cout << "Robot rotating Clockwise" << std::endl;
					robotInputDeviceState->direction = -0.005;
					robotInputDeviceState->moveRequest = true;
					return false;
					break;
				case osgGA::GUIEventAdapter::KEY_Return:
					std::cout << "Lower Arm extending" << std::endl;
					robotInputDeviceState->elevation = -0.005;
					robotInputDeviceState->moveRequest = true;
					return false;
					break;
				case osgGA::GUIEventAdapter::KEY_Shift_R:
					std::cout << "Lower Arm retracting" << std::endl;
					robotInputDeviceState->elevation = 0.005;
					robotInputDeviceState->moveRequest = true;
					return false;
					break;
				default:
					return false;
		}
	}
	case(osgGA::GUIEventAdapter::KEYUP): {
		switch (ea.getKey()) {
		case osgGA::GUIEventAdapter::KEY_Right:
		case osgGA::GUIEventAdapter::KEY_Left:
			std::cout << "Key released" << std::endl;
			robotInputDeviceState->direction = 0.0;
			robotInputDeviceState->moveRequest = false;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_Up:
		case osgGA::GUIEventAdapter::KEY_Down:
			std::cout << "Key released" << std::endl;
			robotInputDeviceState->uaElevation = 0.0;
			robotInputDeviceState->moveRequest = false;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_Return:
		case osgGA::GUIEventAdapter::KEY_Shift_R:
			std::cout << "Key released" << std::endl;
			robotInputDeviceState->elevation = 0.0;
			robotInputDeviceState->moveRequest = false;
			return false;
			break;
		default:
			return false;
		}
	}
	default:
		return false;
	}
}

//function to run before rendering starts -> use this to identify nodes
void init() {
	if (g_pModel) {
		std::cout << "Loaded Model" << std::endl;
	}
	else {
		std::cout << " Failed to load a Model" << std::endl;
	}
}

int main(int argc, char** argv)
{
	osg::ArgumentParser arguments(&argc, argv);

	//to load the model
	g_pModel = osgDB::readNodeFiles(arguments);
	g_pModel->ref();

	if (!g_pModel) return 0;

	init();

	//setup viewer
	osgViewer::Viewer viewer;

	viewer.getCamera()->setClearColor(osg::Vec4(1., 1., 1., 1.)); //setting up the white background

	osg::GraphicsContext::Traits *pTraits = new osg::GraphicsContext::Traits();
	pTraits->x = 20;
	pTraits->y = 20;
	pTraits->width = 600;
	pTraits->height = 480;
	pTraits->windowDecoration = true;
	pTraits->doubleBuffer = true;
	pTraits->sharedContext = 0;

	osg::GraphicsContext *pGC = osg::GraphicsContext::createGraphicsContext(pTraits);
	osgGA::KeySwitchMatrixManipulator* pKeyswitchManipulator = new osgGA::KeySwitchMatrixManipulator();
	pKeyswitchManipulator->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator());
	pKeyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
	pKeyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
	viewer.setCameraManipulator(pKeyswitchManipulator);
	osg::Camera *pCamera = viewer.getCamera();
	pCamera->setGraphicsContext(pGC);
	pCamera->setViewport(new osg::Viewport(0, 0, pTraits->width, pTraits->height));
	pCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	//to add the state manipulator
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

	//to add the thread model handler
	viewer.addEventHandler(new osgViewer::ThreadingHandler);

	//to add the window size toggle handler
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);

	//to add the stats handler
	viewer.addEventHandler(new osgViewer::StatsHandler);

	//to add the record camera path handler
	viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

	//to add the LOD Scale handler
	viewer.addEventHandler(new osgViewer::LODScaleHandler);

	//to add the screen capture handler
	viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

	robotInputDeviceStateType* rIDevState = new robotInputDeviceStateType; //to declare instance of class to record state of keyboard

	//set up the robot update callback pass the constructor a pointer to the robot input device state that is declared above
	findNodeVisitor findNode("BodyRotator");
	g_pModel->accept(findNode);
	osg::MatrixTransform * rotation = dynamic_cast<osg::MatrixTransform *> (findNode.getFirst());
	findNode.getFirst()->setUpdateCallback(new updateRobotPosCallback(rIDevState, "BodyRotator"));

	findNodeVisitor findUpperArm("UpperArmRotator");
	g_pModel->accept(findUpperArm);
	osg::MatrixTransform * uaElevation = dynamic_cast<osg::MatrixTransform *> (findUpperArm.getFirst());
	findUpperArm.getFirst()->setUpdateCallback(new updateRobotPosCallback(rIDevState, "UpperArmRotator"));


	findNodeVisitor findLowerArm("LowerArmRotator");
	g_pModel->accept(findLowerArm);
	osg::MatrixTransform * elevation = dynamic_cast<osg::MatrixTransform *> (findLowerArm.getFirst());
	findLowerArm.getFirst()->setUpdateCallback(new updateRobotPosCallback(rIDevState, "LowerArmRotator"));

	//the constructor for the event handler also gets a pointer to the robot input device state instance
	myKeyboardEventHandler* robotEventHandler = new myKeyboardEventHandler(rIDevState);

	//to add the event handler to the list
	viewer.addEventHandler(robotEventHandler);

	//to set the scene to render & create replica robots in the scene as defined by the static variables below (currently set to one row of 3)
	osg::Group* rootNode = new osg::Group;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 1; j++) {
			osg::MatrixTransform* mt = new osg::MatrixTransform;
			osg::Matrixf location;
			location.setTrans(osg::Vec3d(j * 5, i * 10, 0));
			mt->setMatrix(location);
			mt->addChild(g_pModel);
			rootNode->addChild(mt);
		}
	}

	viewer.addEventHandler(new PickHandler); //to include the outline selection PickHandler

	viewer.setSceneData(rootNode); //to include multiple robot models

	viewer.realize();

	return viewer.run();

	return 0;
}



/*ADDING A SPHERE TEST - DOESNT WORK

#include <osg/ShapeDrawable>

osg::Geode* createSphere() {
osg::Geode* geode = new osg::Geode(); geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(10.0f, 10.0f, 12.0f), 3.0f)));
}

*/



/*BUILDING A WALL TEST - DOESN'T WORK

osg::Geode *g_pWall = 0;
osg::Geometry* pGeoWall = new osg::Geometry;

osg::Vec3Array* pVertsWall = new osg::Vec3Array;
pVertsWall->push_back(osg::Vec3(30, 0, 10));
pVertsWall->push_back(osg::Vec3(30, 0, 0));
pVertsWall->push_back(osg::Vec3(0, 0, 0));
pVertsWall->push_back(osg::Vec3(0, 0, 10));
pGeoWall->setVertexArray(pVertsWall);

osg::DrawElementsUInt* pPrimitiveSetWall = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
pPrimitiveSetWall->push_back(3);
pPrimitiveSetWall->push_back(2);
pPrimitiveSetWall->push_back(1);
pPrimitiveSetWall->push_back(0);
pGeoWall->addPrimitiveSet(pPrimitiveSetWall);

osg::Vec2Array* pTexCoordsWall = new osg::Vec2Array(4);
(*pTexCoordsWall)[0].set(0.0f, 0.0f);
(*pTexCoordsWall)[1].set(1.0f, 0.0f);
(*pTexCoordsWall)[2].set(1.0f, 1.0f);
(*pTexCoordsWall)[3].set(0.0f, 1.0f);
pGeoWall->setTexCoordArray(0, pTexCoordsWall);

osg::Geode* pGeodeWall = new osg::Geode;
osg::StateSet* pStateSetWall = pGeodeWall->getOrCreateStateSet();
pStateSetWall->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
pGeodeWall->addDrawable(pGeoWall);

osg::Texture2D* pTexWall = new osg::Texture2D;
osg::ref_ptr<osg::Image> imgWall = osgDB::readImageFile("../../images/wall.jpg");
osg::ImageStream* isWall = dynamic_cast<osg::ImageStream*>(imgWall.get());
pTexWall->setImage(imgWall.get()); pStateSetWall->setTextureAttributeAndModes(0, pTexWall, osg::StateAttribute::ON);

osg::MatrixTransform* mtWall = new osg::MatrixTransform;
osg::Matrixf locationWall;
locationWall.setTrans(osg::Vec3d(-5, 25, 0));
mtWall->setMatrix(locationWall);
mtWall->addChild(pGeodeWall);
rootNode->addChild(mtWall);

BUILDING A WALL TEST*/