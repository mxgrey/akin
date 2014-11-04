#ifndef KINTREE_H
#define KINTREE_H

#include "akin/AkinIncludes.h"
#include "osgAkin/IncludeOSG.h"
#include "osgAkin/LineTree.h"
#include "osgAkin/KinBranches.h"
#include "osgAkin/Axes.h"

#include <osg/CullFace>

namespace osgAkin {

typedef std::map<akin::Frame*,osg::MatrixTransform*> FrameMtfMap;
typedef std::map<akin::Frame*,osg::Geode*> FrameGeodeMap;
typedef std::map<akin::KinObject*,osg::Group*> ObjectGroupMap;
typedef std::map<osg::Node*,akin::KinObject*> NodeObjectMap;

class KinTree : public osg::Group
{
public:
    KinTree(float line_width=3.0f, float axis_length=0.2f);
    KinTree(akin::Frame& root_frame, float line_width=3.0f, float axis_length=0.2f);
    
    void setRootFrame(akin::Frame& root_frame);
    akin::Frame* getRootFrame();
    
    void update();
    void initialize();

    const NodeObjectMap& getNodeObjectMap() const;
    
protected:
    
    osg::LineWidth* _axisWidth;
    float _axisLength;
    osg::LineWidth* _branchWidth;

    osg::CullFace* _cull;
    
    void _reserveMemory();
    
    osg::Geode* _makeFrameGeode(akin::Frame& frame);
    osg::Group *_makeObjectGroup(akin::KinObject& object);
    
    osg::Geode* _branchGeode;
    KinBranches* _branches;
    
    std::vector<osg::Geode*> _axisGeodes;
    std::vector<osgAkin::Axes*> _axes;
    
//    void _findTrueRoot();
    void _recursiveInitialize(akin::Frame& next_frame);
    void _recursiveUpdate(akin::Frame& next_frame);
    void _renderChildObjects(akin::Frame& frame);
    void _objectInitialize(akin::Frame& frame, akin::KinObject& new_object);
    void _loadVisualArray(const akin::GeometryArray& visuals, osg::Group* group);
    
    akin::Frame* _root;
    FrameMtfMap _frameMtfMap;
    FrameGeodeMap _frameGeodeMap;
    ObjectGroupMap _objectGroupMap;
    NodeObjectMap _nodeObjectMap;

    bool _initialized;
    
};

typedef std::vector<KinTree*> KinTreePtrArray;

} // namespace osgAkin

#endif // KINTREE_H
