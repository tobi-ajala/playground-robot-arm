// raaRobotBuilder.cpp : Defines the entry point for the console application.
//

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
#include <osgViewer/ViewerEventHandlers>

osg::Node *g_pBody=0;
osg::Node *g_pUpperArm=0;
osg::Node *g_pLowerArm=0;
osg::Node *g_pHand=0;
osg::Node *g_pGround=0;
osg::Node *g_pTable1=0;
osg::Node *g_pTable2=0;
osg::Node *g_pObj1=0;
osg::Node *g_pObj2=0;
osg::Node *g_pObj3=0;


int main(int argc, char** argv)
{
	// use an ArgumentParser object to manage the program arguments.
	osg::ArgumentParser arguments(&argc,argv);

	g_pBody= osgDB::readNodeFile(std::string("../../Data/robBody.ac"));
	g_pUpperArm= osgDB::readNodeFile("../../Data/robUpperArm.ac");
	g_pLowerArm= osgDB::readNodeFile("../../Data/robLowerArm.ac");
	g_pHand= osgDB::readNodeFile("../../Data/robHand.ac");
	g_pGround= osgDB::readNodeFile("../../Data/robGround.ac");
	g_pTable1= osgDB::readNodeFile("../../Data/robTable1.ac");
	g_pTable2= osgDB::readNodeFile("../../Data/robTable2.ac");
	g_pObj1= osgDB::readNodeFile("../../Data/robObj1.ac");
	g_pObj2= osgDB::readNodeFile("../../Data/robObj2.ac");
	g_pObj3= osgDB::readNodeFile("../../Data/robObj3.ac");

	osg::MatrixTransform *pBodyRotator=new osg::MatrixTransform();
	osg::MatrixTransform *pBodyLocator=new osg::MatrixTransform();
	osg::MatrixTransform *pWorldLocator=new osg::MatrixTransform();
	osg::MatrixTransform *pTable1Locator=new osg::MatrixTransform();
	osg::MatrixTransform *pTable1TopLocator=new osg::MatrixTransform();
	osg::MatrixTransform *pTable2Locator=new osg::MatrixTransform();
	osg::MatrixTransform *pTable2TopLocator=new osg::MatrixTransform();
	osg::MatrixTransform *pObj1Locator=new osg::MatrixTransform();
	osg::MatrixTransform *pObj2Locator=new osg::MatrixTransform();
	osg::MatrixTransform *pObj3Locator=new osg::MatrixTransform();
	osg::MatrixTransform *pUARotator=new osg::MatrixTransform();
	osg::MatrixTransform *pUALocator=new osg::MatrixTransform();
	osg::MatrixTransform *pLARotator=new osg::MatrixTransform();
	osg::MatrixTransform *pLALocator=new osg::MatrixTransform();
	osg::MatrixTransform *pHandRotator=new osg::MatrixTransform();
	osg::MatrixTransform *pHandLocator=new osg::MatrixTransform();
	osg::MatrixTransform *pHandPositioner=new osg::MatrixTransform();

	pWorldLocator->addChild(pBodyLocator);
	pWorldLocator->addChild(g_pGround);

	pTable1Locator->addChild(g_pTable1);
	pTable1Locator->addChild(pTable1TopLocator);
	pWorldLocator->addChild(pTable1Locator);

	pTable1TopLocator->addChild(pObj1Locator);
	pObj1Locator->addChild(g_pObj1);

	pTable1TopLocator->addChild(pObj2Locator);
	pObj2Locator->addChild(g_pObj2);

	pTable1TopLocator->addChild(pObj3Locator);
	pObj3Locator->addChild(g_pObj3);

	pTable2Locator->addChild(g_pTable2);
	pTable2Locator->addChild(pTable2TopLocator);
	pWorldLocator->addChild(pTable2Locator);

	pBodyLocator->addChild(pBodyRotator);
	pUALocator->addChild(pUARotator);
	pLALocator->addChild(pLARotator);
	pHandLocator->addChild(pHandRotator);
	pHandRotator->addChild(pHandPositioner);

	pBodyRotator->addChild(pUALocator);
	pUARotator->addChild(pLALocator);
	pLARotator->addChild(pHandLocator);

	pBodyRotator->addChild(g_pBody);
	pUARotator->addChild(g_pUpperArm);
	pLARotator->addChild(g_pLowerArm);
	pHandRotator->addChild(g_pHand);

	osg::Matrixf mObj1Loc, mObj2Loc, mObj3Loc;
	mObj1Loc.makeTranslate(osg::Vec3f(0.2f, 0.2f, 0.0f));
	mObj2Loc.makeTranslate(osg::Vec3f(0.8f, -0.3f, 0.0f));
	mObj3Loc.makeTranslate(osg::Vec3f(-0.6f, 0.3f, 0.0f));

	pObj1Locator->setMatrix(mObj1Loc);
	pObj2Locator->setMatrix(mObj2Loc);
	pObj3Locator->setMatrix(mObj3Loc);

	osg::Matrixf mT1Loc, mT1LocTop;
	mT1Loc.makeTranslate(osg::Vec3f(3.0f, 2.0f, 0.0f));
	mT1LocTop.makeTranslate(osg::Vec3f(0.0f, 0.0f, 1.0f));
	pTable1Locator->setMatrix(mT1Loc);
	pTable1TopLocator->setMatrix(mT1LocTop);

	osg::Matrixf mT2Loc, mT2LocTop;
	mT2Loc.makeTranslate(osg::Vec3f(3.0f, -2.0f, 0.0f));
	mT2LocTop.makeTranslate(osg::Vec3f(0.0f, 0.0f, 1.0f));
	pTable2Locator->setMatrix(mT2Loc);
	pTable2TopLocator->setMatrix(mT2LocTop);

	osg::Matrixf mUARot, mUALoc;
	mUARot.makeRotate(osg::DegreesToRadians(-20.0f), osg::Vec3f(0.0f, 1.0f, 0.0f));
	mUALoc.makeTranslate(osg::Vec3f(0.0f, 0.0f, 1.8f));
	pUARotator->setMatrix(mUARot);
	pUALocator->setMatrix(mUALoc);

	osg::Matrixf mLARot, mLALoc;
	mLARot.makeRotate(osg::DegreesToRadians(45.0f), osg::Vec3f(0.0f, 1.0f, 0.0f));
	mLALoc.makeTranslate(osg::Vec3f(2.4f, 0.0f, 0.0f));
	pLARotator->setMatrix(mLARot);
	pLALocator->setMatrix(mLALoc);

	osg::Matrixf mHandRot, mHandLoc;
	mHandRot.makeRotate(osg::DegreesToRadians(-20.0f), osg::Vec3f(0.0f, 1.0f, 0.0f));
	mHandLoc.makeTranslate(osg::Vec3f(2.6f, 0.0f, 0.0f));
	pHandRotator->setMatrix(mHandRot);
	pHandLocator->setMatrix(mHandLoc);

	pBodyRotator->setName("BodyRotator");
	pBodyRotator->setDataVariance(osg::Object::DYNAMIC);
	pBodyLocator->setName("BodyLocator");
	pBodyLocator->setDataVariance(osg::Object::DYNAMIC);
	g_pBody->setName("model");

	g_pObj1->setName("model");
	g_pObj2->setName("model");
	g_pObj3->setName("model");

	pObj1Locator->setName("Object1Locator");
	pObj2Locator->setName("Object2Locator");
	pObj3Locator->setName("Object3Locator");

	pObj1Locator->setDataVariance(osg::Node::DYNAMIC);
	pObj2Locator->setDataVariance(osg::Node::DYNAMIC);
	pObj3Locator->setDataVariance(osg::Node::DYNAMIC);

	g_pTable1->setName("model");
	pTable1TopLocator->setName("Table1TopLocator");
	pTable1Locator->setName("Table1Locator");

	g_pTable2->setName("model");
	pTable2TopLocator->setName("Table2TopLocator");
	pTable2Locator->setName("Table2Locator");

	pUARotator->setName("UpperArmRotator");
	pUARotator->setDataVariance(osg::Object::DYNAMIC);
	pUALocator->setName("UpperArmLocator");
	g_pUpperArm->setName("model");

	pLARotator->setName("LowerArmRotator");
	pLARotator->setDataVariance(osg::Object::DYNAMIC);
	pLALocator->setName("LowerArmLocator");
	g_pLowerArm->setName("model");

	pHandRotator->setName("HandRotator");
	pHandRotator->setDataVariance(osg::Object::DYNAMIC);
	pHandLocator->setName("HandLocator");
	g_pHand->setName("model");
	pHandPositioner->setName("HandPositioner");
	pHandPositioner->setDataVariance(osg::Object::DYNAMIC);

	osgViewer::Viewer viewer;

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
	pKeyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
	pKeyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
	pKeyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
	viewer.setCameraManipulator(pKeyswitchManipulator);
	osg::Camera *pCamera = viewer.getCamera();
	pCamera->setGraphicsContext(pGC);
	pCamera->setViewport(new osg::Viewport(0,0, pTraits->width, pTraits->height));


	// add the state manipulator
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

	// add the thread model handler
	viewer.addEventHandler(new osgViewer::ThreadingHandler);

	// add the window size toggle handler
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);

	// add the stats handler
	viewer.addEventHandler(new osgViewer::StatsHandler);

	// add the record camera path handler
	viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

	// add the LOD Scale handler
	viewer.addEventHandler(new osgViewer::LODScaleHandler);

	// add the screen capture handler
	viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);


	osgDB::writeObjectFile(*pWorldLocator, std::string("../../Data/robot.osg"));
 
	// set the scene to render
	viewer.setSceneData(pWorldLocator);

	viewer.realize();

	return viewer.run();
}

