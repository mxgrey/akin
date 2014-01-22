
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
    for(size_t i=0; i<_kinTrees.size(); ++i)
        if(_kinTrees[i]->getRootFrame() == &new_root_frame)
            return;
    
    KinBranches* new_tree = new KinBranches(new_root_frame);
    _kinTrees.push_back(new_tree);
    
    addDrawable(new_tree);
}

void AkinNode::removeRootFrame(akin::Frame &existing_frame)
{
    for(size_t i=0; i<_kinTrees.size(); ++i)
    {
        if(&existing_frame == _kinTrees[i]->getRootFrame())
        {
            removeDrawable(_kinTrees[i]);
            _kinTrees.erase(_kinTrees.begin()+i);
        }
    }
}

void AkinNode::update()
{
    for(size_t i=0; i<_kinTrees.size(); ++i)
    {
        _kinTrees[i]->update();
    }
}

void AkinNode::initialize(size_t tree_num)
{
    
}

