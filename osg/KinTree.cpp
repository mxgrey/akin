
#include "KinTree.h"

using namespace osgAkin;
using namespace akin;

KinTree::KinTree(float line_width, float axis_length) :
    _initialized(false),
    _root(NULL),
    _axisWidth(new osg::LineWidth(line_width)),
    _axisLength(axis_length),
    _branchWidth(new osg::LineWidth(1.0f))
{
    _reserveMemory();
    _branchGeode->addDrawable(_branches);
}

KinTree::KinTree(Frame &root_frame, float line_width, float axis_length) :
    _initialized(false),
    _root(NULL),
    _axisWidth(new osg::LineWidth(line_width)),
    _axisLength(axis_length),
    _branchWidth(new osg::LineWidth(1.0f))
{
    _reserveMemory();
    setRootFrame(root_frame);
}

void KinTree::_reserveMemory()
{
    _cull = new osg::CullFace;
    _cull->setMode(osg::CullFace::BACK);

    _branchGeode = new osg::Geode;
    addChild(_branchGeode);
    _branchGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    _branchGeode->getOrCreateStateSet()->setAttributeAndModes(_branchWidth);

    _branches = new KinBranches;
    _branchGeode->addDrawable(_branches);
}

void KinTree::setRootFrame(Frame &root_frame)
{
    if(&root_frame == _root)
        return;
    
    _root = &root_frame;
    _findTrueRoot();
    
    _branches->setRootFrame(*_root);
    
    _frameMap.erase(_frameMap.begin(),_frameMap.end());

    _initialized = false;
}

Frame *KinTree::getRootFrame()
{
    _findTrueRoot();
    return _root;
}

void KinTree::update()
{
    if(!_initialized)
    {
        initialize();
        return;
    }
    
    _findTrueRoot();
    _recursiveUpdate(*_root);
    _branches->update();
}

void KinTree::initialize()
{
    _frameMap.erase(_frameMap.begin(),_frameMap.end());
    _findTrueRoot();
    _recursiveInitialize(*_root);
    _branches->initialize();
    _initialized = true;
}

void KinTree::_findTrueRoot()
{
    while(!_root->refFrame().isWorld())
        _root = &_root->refFrame();
}

void KinTree::_recursiveUpdate(Frame &next_frame)
{
    FrameMtfMap::const_iterator m = _frameMap.find(&next_frame);
    if( m != _frameMap.end() )
    {
        _frameMap[&next_frame]->setMatrix(cosg(next_frame.respectToWorld()));
        
        for(size_t i=0; i<next_frame.numChildFrames(); ++i)
        {
            _recursiveUpdate(next_frame.childFrame(i));
        }
    }
    else
    {
        _recursiveInitialize(next_frame);
    }
    
}

void KinTree::_recursiveInitialize(Frame &next_frame)
{
    _frameMap[&next_frame] = new osg::MatrixTransform;
    addChild(_frameMap[&next_frame]);
    _frameMap[&next_frame]->addChild(_makeAxisGeode());
    _frameMap[&next_frame]->setMatrix(cosg(next_frame.respectToWorld()));
    
    for(size_t i=0; i<next_frame.numChildFrames(); ++i)
    {
        _recursiveUpdate(next_frame.childFrame(i));
    }
}

osg::Geode* KinTree::_makeAxisGeode()
{
    osg::Geode* axisGeode = new osg::Geode;
    _axisGeodes.push_back(axisGeode);
    
    Axes* axes = new Axes(_axisLength);
    _axes.push_back(axes);
    
    axisGeode->addDrawable(axes);
    axisGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    axisGeode->getOrCreateStateSet()->setAttributeAndModes(_axisWidth);
    axisGeode->getOrCreateStateSet()->setAttributeAndModes(_cull, osg::StateAttribute::ON);
    axisGeode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    axisGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    
    return axisGeode;
}








