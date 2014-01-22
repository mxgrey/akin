#ifndef AKINNODE_H
#define AKINNODE_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"
#include "KinBranches.h"

namespace osgAkin {


class AkinNode : public osg::Geode
{
public:

    AkinNode();

    void addRootFrame(akin::Frame& new_root_frame);

    void removeRootFrame(akin::Frame& existing_frame);
    
    virtual void update();

    virtual void initialize(size_t tree_num);

protected:
    
    KinBranchesPtrArray _kinTrees;
    
    osg::LineWidth* _linewidth;
    bool _initialized;
};

} // osgAkin

#endif // AKINNODE_H
