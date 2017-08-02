#include "Main.h"

int main(int argc, char** argv)
{
	//Create recorder
	RecordPlayback = &(osg_Recorder::GetRecorder());

	osg::ArgumentParser arguments(&argc, argv);

	//to load the model
	g_pModel = osgDB::readNodeFiles(arguments);
	g_pModel->ref();

	if (!g_pModel) return 0;

	init();

	//setup viewer
	osgViewer::Viewer viewer;

	viewer.getCamera()->setClearColor(osg::Vec4(1., 1., 1., 1.));

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

	//add sun at specified location
	AddSun(rootNode, osg::Vec3f(20.0f, 0.0f, 15.0f));
	//add walls at specified location
	AddWalls(rootNode, osg::Vec2(-10,-10), osg::Vec2(11,28));

	//disable default headlight
	viewer.setLightingMode(osg::View::NO_LIGHT);

	viewer.addEventHandler(new PickHandler); //to include the outline selection PickHandler

	viewer.setSceneData(rootNode); //to include multiple robot models

	viewer.realize();

	return viewer.run();

	return 0;
}