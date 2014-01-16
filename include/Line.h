#ifndef OSGLINE_H
#define OSGLINE_H

#include <AkinIncludes.h>
#include <osg/Geometry>

namespace akin_osg {

class Line : public osg::Geometry
{
public:
    
    Line(akin::KinTranslation& start_point,
         akin::KinTranslation& end_point);
    
};


} // namespace akin_osg

#endif // OSGLINE_H
