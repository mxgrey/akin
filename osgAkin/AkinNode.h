#ifndef AKINNODE_H
#define AKINNODE_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"
#include "AkinGeometry.h"

namespace osgAkin {

typedef std::map<akin::Frame*,ushort> FrameIndexMap;

class AkinNode : public osg::Node
{
public:



    inline void addFrame(akin::Frame& new_frame)
    {
        for(size_t i=0; i<_frames.size(); ++i)
            if(&new_frame == _frames[i])
                return;

        _frames.push_back(&new_frame);
    }

    inline void removeFrame(akin::Frame& existing_frame)
    {
        for(size_t i=0; i<_frames.size(); ++i)
        {
            if(&existing_frame == _frames[i])
            {
                _frames.erase(_frames.begin()+i);
                break;
            }
        }
    }

protected:

    akin::FramePtrArray _frames;
    akin::KinObjectPtrArray _kobjects;

    FrameIndexMap _index;

    AkinGeometry* _geom;
};

} // osgAkin

#endif // AKINNODE_H
