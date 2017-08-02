#ifndef H_OSG_UTIL_CODE
#define H_OSG_UTIL_CODE

#include <osg/Light>
#include <osg/LightSource>
#include <osg/geode>
#include <osg/ShapeDrawable>
#include <osg/material>

void AddSun(osg::Group * root, osg::Vec3f Location);
void AddWalls(osg::Group * root, osg::Vec2 TopLeft, osg::Vec2 BottomRight);

#endif
