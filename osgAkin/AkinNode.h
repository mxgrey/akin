#ifndef AKINNODE_H
#define AKINNODE_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"
#include "AkinGeometry.h"

namespace osgAkin {

class AkinNode : public osg::Node
{
public:
    AkinNode();

protected:
    AkinGeometryPtrArray geoms;
};

} // osgAkin

#endif // AKINNODE_H
