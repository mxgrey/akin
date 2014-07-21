
#include "osgAkin/KinTree.h"
#include "osgDB/ReadFile"
#include "osg/ShapeDrawable"
#include "osgAkin/AkinVisual.h"

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
    
    _frameMtfMap.erase(_frameMtfMap.begin(),_frameMtfMap.end());

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
    _frameMtfMap.erase(_frameMtfMap.begin(),_frameMtfMap.end());
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
    FrameMtfMap::const_iterator m = _frameMtfMap.find(&next_frame);
    if( m != _frameMtfMap.end() )
    {
        _frameMtfMap[&next_frame]->setMatrix(cosg(next_frame.respectToWorld()));
        _renderChildObjects(next_frame);
        
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
    _frameMtfMap[&next_frame] = new osg::MatrixTransform;
    addChild(_frameMtfMap[&next_frame]);
    _frameMtfMap[&next_frame]->addChild(_makeFrameGeode(next_frame));
    _frameMtfMap[&next_frame]->setMatrix(cosg(next_frame.respectToWorld()));
    
    for(size_t i=0; i<next_frame.numChildFrames(); ++i)
    {
        _recursiveUpdate(next_frame.childFrame(i));
    }
}

void KinTree::_renderChildObjects(Frame &frame)
{
    if(frame.visualsChanged())
    {
        _loadVisualArray(frame.grabVisualsAndReset(),
                         _frameMtfMap[&frame]);
    }

    for(size_t i=0; i<frame.numChildObjects(); ++i)
    {
        if(!frame.childObject(i).isFrame())
        {
            ObjectGroupMap::const_iterator m = _objectGroupMap.find(&frame.childObject(i));
            if( m != _objectGroupMap.end() )
            {
                if(frame.childObject(i).visualsChanged())
                {
                    _loadVisualArray(frame.childObject(i).grabVisualsAndReset(),
                                     _objectGroupMap[&frame]);
                }
            }
            else
            {
                _objectInitialize(frame, frame.childObject(i));
            }
        }
    }
}

void KinTree::_objectInitialize(Frame &frame, KinObject &new_object)
{
    _frameMtfMap[&frame]->addChild(_makeObjectGroup(new_object));
    _loadVisualArray(new_object.grabVisualsAndReset(),_objectGroupMap[&new_object]);
}

void KinTree::_loadVisualArray(const GeometryArray &visuals, osg::Group *group)
{
    group->removeChildren(0, group->getNumChildren());

    for(size_t i=0; i<visuals.size(); ++i)
    {
        group->addChild(new AkinVisual(visuals[i]));
    }
}

osg::Geode* KinTree::_makeFrameGeode(Frame &frame)
{
    osg::Geode* frameGeode = new osg::Geode;
    _axisGeodes.push_back(frameGeode);
    
    Axes* axes = new Axes(_axisLength);
    _axes.push_back(axes);
    
    frameGeode->addDrawable(axes);
    frameGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    frameGeode->getOrCreateStateSet()->setAttributeAndModes(_axisWidth);
    frameGeode->getOrCreateStateSet()->setAttributeAndModes(_cull, osg::StateAttribute::ON);
    frameGeode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    frameGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    
    _frameGeodeMap[&frame] = frameGeode;

    return frameGeode;
}

osg::Group* KinTree::_makeObjectGroup(KinObject &object)
{
    osg::Group* objectGroup = new osg::Group;
    _objectGroupMap[&object] = objectGroup;
    return objectGroup;
}






