#ifndef AKINGEOMETRY_H
#define AKINGEOMETRY_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"

namespace osgAkin {

class AkinGeometry : public osg::Geometry
{
public:

    inline void addFrame(akin::Frame& new_frame)
    {
        for(size_t i=0; i<frames.size(); ++i)
            if(&new_frame == frames[i])
                return;

        frames.push_back(&new_frame);
    }

    inline void removeFrame(akin::Frame& existing_frame)
    {
        for(size_t i=0; i<frames.size(); ++i)
        {
            if(&existing_frame == frames[i])
            {
                frames.erase(frames.begin()+i);
                break;
            }
        }
    }

    virtual void updateFrames();

protected:

    akin::FramePtrArray frames;
    akin::KinObjectPtrArray kobjects;

};

typedef std::vector<osgAkin::AkinGeometry*> AkinGeometryPtrArray;

} // namespace osgAkin

#endif // AKINGEOMETRY_H
