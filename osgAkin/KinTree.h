#ifndef KINTREE_H
#define KINTREE_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"
#include "LineTree.h"
#include "KinBranches.h"
#include "Axes.h"

#include <osg/CullFace>

namespace osgAkin {

typedef std::map<akin::Frame*,osg::MatrixTransform*> FrameMtfMap;

class KinTree : public osg::Group
{
public:
    KinTree(float line_width=3.0f, float axis_length=0.2f);
    KinTree(akin::Frame& root_frame, float line_width=3.0f, float axis_length=0.2f);
    
    void setRootFrame(akin::Frame& root_frame);
    const akin::Frame* getRootFrame();
    
    void update();
    void initialize();
    
protected:
    
    osg::LineWidth* _lineWidth;
    float _axisLength;

    osg::CullFace* _cull;
    
    void _reserveMemory();
    
    osg::Geode* _makeAxisGeode();
    
    osg::Geode* _branchGeode;
    KinBranches* _branches;
    
    std::vector<osg::Geode*> _axisGeodes;
    std::vector<osgAkin::Axes*> _axes;
    
    void _findTrueRoot();
    void _recursiveInitialize(akin::Frame& next_frame);
    void _recursiveUpdate(akin::Frame& next_frame);
    
    akin::Frame* _root;
    FrameMtfMap _frameMap;
    bool _initialized;
    
};

} // namespace osgAkin

#endif // KINTREE_H
