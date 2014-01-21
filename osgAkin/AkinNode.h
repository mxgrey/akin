#ifndef AKINNODE_H
#define AKINNODE_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"
//#include "AkinGeometry.h"
#include "AkinLineGraph.h"

namespace osgAkin {

typedef std::map<akin::Frame*,ushort> FrameIndexMap;

class AkinNode : public osg::Geode
{
public:
    
    inline AkinNode() : _initialized(false)
    {
//        addDrawable(_lineTrees[0]);
    }

    inline void addRootFrame(akin::Frame& new_root_frame)
    {
        for(size_t i=0; i<_frames.size(); ++i)
            if(&new_root_frame == _frames[i])
                return;

        _frames.push_back(&new_root_frame);
        
        _initialized = false;
    }

    inline void removeRootFrame(akin::Frame& existing_frame)
    {
        for(size_t i=0; i<_frames.size(); ++i)
        {
            if(&existing_frame == _frames[i])
            {
                _frames.erase(_frames.begin()+i);
                break;
            }
        }
        
        _initialized = false;
    }
    
    inline virtual void update()
    {
        if(!_initialized)
        {
            initialize();
            return;
        }
        
        
    }
    
    inline virtual void initialize()
    {
//        _lineTrees[0]->clear();
        
        
        
        
        
        _initialized = true;
    }

protected:
    
    void _recursiveFrameInit(akin::Frame& nextFrame)
    {
        
    }
    
    bool _initialized;

    akin::FramePtrArray _frames;
    akin::KinObjectPtrArray _kobjects;

    FrameIndexMap _index;

    AkinLineTreePtrArray _lineTrees;
    
};

} // osgAkin

#endif // AKINNODE_H
