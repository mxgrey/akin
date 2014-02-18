#ifndef KINBRANCHES_H
#define KINBRANCHES_H


#include "akin/AkinIncludes.h"
#include "osgAkin/IncludeOSG.h"
#include "osgAkin/LineTree.h"

namespace osgAkin {

typedef std::map<akin::Frame*,ushort> FrameIndexMap;

class KinBranches : public LineTree
{
public:
    
    KinBranches();
    KinBranches(akin::Frame& root_frame);
    
    void setRootFrame(akin::Frame& root_frame);
    const akin::Frame* getRootFrame();
    
    void update();
    void initialize();
    
protected:
    
    void _findTrueRoot();
    void _recursiveInitialize(akin::Frame& next_frame, ushort parent_num);
    void _recursiveUpdate(akin::Frame& next_frame);
    
    akin::Frame* _root;
    FrameIndexMap _indices;
    bool _initialized;
};

} // namespace osgAkin

#endif // KINBRANCHES_H
