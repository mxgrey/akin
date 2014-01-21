#ifndef AKINNODE_H
#define AKINNODE_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"
#include "LineTree.h"

namespace osgAkin {

typedef std::map<akin::Frame*,ushort> FrameIndexMap;

class AkinNode : public osg::Geode
{
public:

    AkinNode();

    void addRootFrame(akin::Frame& new_root_frame);

    void removeRootFrame(akin::Frame& existing_frame);
    
    virtual void update();

    virtual void initialize(size_t tree_num);

protected:
    
    void _recursiveFrameInitialize(akin::Frame& next_frame, ushort parent_num, ushort tree_num);
    void _recursiveUpdate(akin::Frame& next_frame, ushort tree_num);

    akin::FramePtrArray _frames;
    std::vector<FrameIndexMap> _indices;
    std::vector<bool> _initialized;
    LineTreePtrArray _lineTrees;

    osg::LineWidth* _linewidth;
    
};

} // osgAkin

#endif // AKINNODE_H
