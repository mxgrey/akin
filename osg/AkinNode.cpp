
#include "AkinCallback.h"

using namespace osgAkin;
using namespace akin;
using namespace std;

AkinNode::AkinNode()
{
    setUpdateCallback(new osgAkin::AkinCallback);

    getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    _linewidth = new osg::LineWidth;
    _linewidth->setWidth(2.0f);
    getOrCreateStateSet()->setAttributeAndModes(_linewidth);
}

void AkinNode::addRootFrame(akin::Frame &new_root_frame)
{
    for(size_t i=0; i<_frames.size(); ++i)
        if(&new_root_frame == _frames[i])
            return;

    _frames.push_back(&new_root_frame); // _frames
    LineTree* new_tree = new LineTree(new_root_frame.respectToWorld().translation());
    _lineTrees.push_back(new_tree);     // _lineTrees
    FrameIndexMap new_indices;
    new_indices[&new_root_frame] = 0;
    _indices.push_back(new_indices);    // _indices

    _initialized.push_back(false);      // _initialized
}

void AkinNode::removeRootFrame(akin::Frame &existing_frame)
{
    for(size_t i=0; i<_frames.size(); ++i)
    {
        if(&existing_frame == _frames[i])
        {
            _frames.erase(_frames.begin()+i);
            _lineTrees.erase(_lineTrees.begin()+i);
            _indices.erase(_indices.begin()+i);
            _initialized.erase(_initialized.begin()+i);

            break;
        }
    }
}

void AkinNode::update()
{
    for(size_t i=0; i<_lineTrees.size(); ++i)
    {
        if(!_initialized[i])
        {
            initialize(i);
            return;
        }

        _recursiveUpdate(*(_frames[i]), i);

        _lineTrees[i]->updateVertices();
    }
}

void AkinNode::_recursiveUpdate(Frame &next_frame, ushort tree_num)
{
    LineTree* tree = _lineTrees[tree_num];
    FrameIndexMap& indices = _indices[tree_num];
    tree->moveVertex(indices[&next_frame], next_frame.respectToWorld().translation());

    for(size_t i=0; i<next_frame.numChildFrames(); ++i)
    {
        Frame& next_child = next_frame.childFrame(i);
        FrameIndexMap::const_iterator v = indices.find(&next_child);
        if(v != indices.end())
        {
            _recursiveUpdate(next_child, tree_num);
        }
        else
        {
            indices[&next_child] = tree->addVertex(next_child.respectToWorld().translation(), indices[&next_frame]);
            _recursiveFrameInitialize(next_child, indices[&next_child], tree_num);
        }
    }
}

void AkinNode::initialize(size_t tree_num)
{
    _lineTrees[tree_num]->clear();
    _recursiveFrameInitialize(*(_frames[tree_num]), 0, tree_num);

    if(_lineTrees[tree_num]->vertexCount()>1)
    {
        _initialized[tree_num] = true;
        addDrawable(_lineTrees[tree_num]);
    }
}

void AkinNode::_recursiveFrameInitialize(akin::Frame &next_frame, ushort parent_num, ushort tree_num)
{
    LineTree* tree = _lineTrees[tree_num];
    FrameIndexMap& indices = _indices[tree_num];

    for(size_t i=0; i<next_frame.numChildFrames(); ++i)
    {
        Frame& next_child = next_frame.childFrame(i);

        indices[&next_child] = tree->addVertex(next_child.respectToWorld().translation(), parent_num);
        _recursiveFrameInitialize(next_child, indices[&next_child], tree_num);
    }
}
