#include "Utilities_code.h"

void AddSun(osg::Group * root, osg::Vec3f Location)
{
	osg::ref_ptr<osg::Group> lightGroup(new osg::Group);
	osg::ref_ptr<osg::StateSet> lightSS(root->getOrCreateStateSet());
	osg::ref_ptr<osg::LightSource> lightSource1 = new osg::LightSource;
	osg::ref_ptr<osg::LightSource> lightSource2 = new osg::LightSource;


	//set light location
	osg::Vec4f lightPosition(Location, 1.0f);

	// create a local light.
	osg::ref_ptr<osg::Light> sunLight = new osg::Light;
	sunLight->setLightNum(1);
	sunLight->setPosition(lightPosition);

	sunLight->setAmbient(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	sunLight->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	sunLight->setSpecular(osg::Vec4(1, 1, 1, 1));  // some examples don't have this one
	sunLight->setConstantAttenuation(1.0f);
	sunLight->setDirection(osg::Vec3(0.0f, 0.0f, -1.0f));
	lightSource1->setLight(sunLight.get());

	lightSource1->setLocalStateSetModes(osg::StateAttribute::ON);
	lightSource1->setStateSetModes(*lightSS, osg::StateAttribute::ON);

	lightGroup->addChild(lightSource1.get());

	//create a simple marker geode
	osg::ref_ptr<osg::Geode> lightMarkerGeode(new osg::Geode);
	osg::ref_ptr<osg::ShapeDrawable> Draw = new osg::ShapeDrawable(new osg::Sphere(Location, 1.0f));
	lightMarkerGeode->addDrawable(Draw);

	//set sun colour to orange
	Draw->setColor(osg::Vec4(1.0f, 0.6f, 0.3f,1.0f));


	//attach light to group
	root->addChild(lightGroup.get());
	root->addChild(lightMarkerGeode.get());
}

void AddWalls(osg::Group * root, osg::Vec2 TopLeft, osg::Vec2 BottomRight)
{

	//calulate wall length using absolute values (avoid negative errors)
	float xLength = abs(TopLeft.x()) + abs(BottomRight.x());
	float yLength = abs(TopLeft.y()) + abs(BottomRight.y());
	float zLength = 2;

	//calulate center points
	float xCenter = (TopLeft.x() + BottomRight.x()) / 2;
	float yCenter = (TopLeft.y() + BottomRight.y()) / 2;
	float zCenter = zLength / 2;



	//calulate center point of each wall
	osg::Vec3f Top_Wall_Center(xCenter, TopLeft.y(), zCenter);
	osg::Vec3f Bottom_Wall_Center(xCenter, BottomRight.y(), zCenter);
	osg::Vec3f Left_Wall_Center(TopLeft.x(), yCenter, zCenter);
	osg::Vec3f Right_Wall_Center(BottomRight.x(), yCenter, zCenter);
	
	//Create geode objects
	osg::ref_ptr<osg::Geode> Top_WallGeode(new osg::Geode);
	osg::ref_ptr<osg::Geode> Bottom_WallGeode(new osg::Geode);
	osg::ref_ptr<osg::Geode> Left_WallGeode(new osg::Geode);
	osg::ref_ptr<osg::Geode> Right_WallGeode(new osg::Geode);

	osg::ref_ptr<osg::ShapeDrawable> Top_Wall_Draw = new osg::ShapeDrawable(new osg::Box(Top_Wall_Center, xLength,1.0f, zLength));
	Top_WallGeode->addDrawable(Top_Wall_Draw);
	osg::ref_ptr<osg::ShapeDrawable> Bottom_Wall_Draw = new osg::ShapeDrawable(new osg::Box(Bottom_Wall_Center, xLength, 1.0f, zLength));
	Bottom_WallGeode->addDrawable(Bottom_Wall_Draw);
	osg::ref_ptr<osg::ShapeDrawable> Left_Wall_Draw = new osg::ShapeDrawable(new osg::Box(Left_Wall_Center, 1.0f, yLength, zLength));
	Left_WallGeode->addDrawable(Left_Wall_Draw);
	osg::ref_ptr<osg::ShapeDrawable> Right_Wall_Draw = new osg::ShapeDrawable(new osg::Box(Right_Wall_Center, 1.0f, yLength, zLength));
	Right_WallGeode->addDrawable(Right_Wall_Draw);

	
	//create vec4 to hold grey-colour
	osg::Vec4f Grey(0.8f, 0.8f, 0.8f, 1.0f);

	//apply colour to walls
	Top_Wall_Draw->setColor(Grey);
	Bottom_Wall_Draw->setColor(Grey);
	Left_Wall_Draw->setColor(Grey);
	Right_Wall_Draw->setColor(Grey);

	//add walls to scene
	root->addChild(Top_WallGeode.get());
	root->addChild(Bottom_WallGeode.get());
	root->addChild(Left_WallGeode.get());
	root->addChild(Right_WallGeode.get());

}
