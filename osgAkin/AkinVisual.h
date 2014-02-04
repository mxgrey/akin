#ifndef AKINVISUAL_H
#define AKINVISUAL_H

#include "IncludeOSG.h"
#include "Geometry.h"


namespace osgAkin {

class AkinVisual : public osg::MatrixTransform
{
public:

    AkinVisual(const akin::Geometry& visual = akin::Geometry::Empty());

protected:

    void _initializeVisual(const akin::Geometry& visual);

    osg::Geode* _makeSphere(const akin::Geometry& visual);
    osg::Geode* _makeBox(const akin::Geometry& visual);

};


} // namespace osgAkin


#endif // AKINVISUAL_H
