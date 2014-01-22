#ifndef KINBRANCHES_H
#define KINBRANCHES_H


#include "AkinIncludes.h"
#include "IncludeOSG.h"
#include "LineTree.h"

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
    
    void _recursiveInitialize(akin::Frame& next_frame, ushort parent_num);
    void _recursiveUpdate(akin::Frame& next_frame);
    
    akin::Frame* _root;
    FrameIndexMap _indices;
    bool _initialized;
};

typedef std::vector<KinBranches*> KinBranchesPtrArray;

} // namespace osgAkin

#endif // KINBRANCHES_H
