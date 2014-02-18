#ifndef AKINVISUAL_H
#define AKINVISUAL_H

#include "osgAkin/IncludeOSG.h"
#include "akin/Geometry.h"
#include <osg/CullFace>

inline osg::Vec4 cosg(const akin::ColorSpec& color)
{
    osg::Vec4 osgColor;
    osgColor.r() = color.array[0];
    osgColor.g() = color.array[1];
    osgColor.b() = color.array[2];
    osgColor.a() = color.array[3];
    return osgColor;
}

namespace osgAkin {

class AkinVisual : public osg::MatrixTransform
{
public:

    AkinVisual(const akin::Geometry& visual = akin::Geometry::Empty());

protected:

    void _initializeVisual(const akin::Geometry& visual);

//    osg::Geode* _makeSphere(const akin::Geometry& visual);
    osg::Geode* _makeBox(const akin::Geometry& visual);
    osg::Geode* _makeAxes(const akin::Geometry& visual);

    void _setGeodeModes(osg::Geode* geode);
    
    osg::Vec4Array* _colors;
    osg::LineWidth* _lineWidth;
    osg::CullFace* _cull;
};


} // namespace osgAkin


#endif // AKINVISUAL_H
