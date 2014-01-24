
#include "KinBranches.h"

using namespace osgAkin;
using namespace akin;

KinBranches::KinBranches() :
    _initialized(false),
    _root(NULL)
{
    
}

KinBranches::KinBranches(akin::Frame &root_frame) :
    LineTree(root_frame.respectToWorld().translation()),
    _initialized(false),
    _root(NULL)
{
    setRootFrame(root_frame);
}

void KinBranches::setRootFrame(akin::Frame &root_frame)
{
    if(&root_frame == _root)
        return;
    
    _root = &root_frame;
    _findTrueRoot();
    
    _indices.erase(_indices.begin(),_indices.end());
    
    _initialized = false;
}

const akin::Frame *KinBranches::getRootFrame()
{
    _findTrueRoot();
    return _root;
}


void KinBranches::update()
{
    if(!_initialized)
    {
        initialize();
        return;
    }
    
    _findTrueRoot();
    
    _recursiveUpdate(*_root);
    
    updateVertices();
}

void KinBranches::initialize()
{
    clear();
    _findTrueRoot();
    _recursiveInitialize(*_root, 0);
    _initialized = true;
}

void KinBranches::_findTrueRoot()
{
    while(!_root->refFrame().isWorld())
        _root = &_root->refFrame();
}

void KinBranches::_recursiveUpdate(akin::Frame &next_frame)
{
    moveVertex(_indices[&next_frame], next_frame.respectToWorld().translation());
    
    for(size_t i=0; i<next_frame.numChildFrames(); ++i)
    {
        Frame& next_child = next_frame.childFrame(i);
        FrameIndexMap::const_iterator v = _indices.find(&next_child);
        if( v != _indices.end() )
        {
            _recursiveUpdate(next_child);
        }
        else
        {
            _recursiveInitialize(next_child, _indices[&next_frame]);
        }
    }
}

void KinBranches::_recursiveInitialize(akin::Frame &next_frame, ushort parent_num)
{
    _indices[&next_frame] = addVertex(next_frame.respectToWorld().translation(), parent_num);
    
    for(size_t i=0; i<next_frame.numChildFrames(); ++i)
    {
        Frame& next_child = next_frame.childFrame(i);
        
        _recursiveInitialize(next_child, _indices[&next_frame]);
    }
}
