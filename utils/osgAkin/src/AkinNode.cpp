/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Jan 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Humanoid Robotics Lab nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "osgAkin/AkinCallback.h"

using namespace osgAkin;
using namespace akin;
using namespace std;

AkinNode::AkinNode()
{
    setUpdateCallback(new osgAkin::AkinCallback);
    _dummyFrame = new Frame(Transform::Identity());
    _dummyFrame->name("dummy");
    _dummyRobot = new Robot;
    _dummyRobot->name("dummy");
}

akin::Frame* AkinNode::_findTrueRoot(Frame &some_frame)
{
    Frame* true_root = &some_frame;
    while(!true_root->refFrame().isWorld())
        true_root = &true_root->refFrame();
    return true_root;
}

size_t AkinNode::addRootFrame(akin::Frame &new_root_frame)
{
    Frame* true_root = _findTrueRoot(new_root_frame);
    for(size_t i=0; i<_kinTrees.size(); ++i)
        if(_kinTrees[i]->getRootFrame() == true_root)
            return i;
    
    KinTree* new_tree = new KinTree(*true_root);
    _kinTrees.push_back(new_tree);
    
    addChild(new_tree);

    _frames.push_back(&new_root_frame);
    return _frames.size()-1;
}

size_t AkinNode::addRobot(Robot &new_robot)
{
//    addRootFrame(new_robot.link(0));

    for(size_t i=0; i<_robots.size(); ++i)
        if(_robots[i] == &new_robot)
            return i;

    _robots.push_back(&new_robot);
    return _robots.size()-1;
}

void AkinNode::removeRootFrame(akin::Frame &existing_frame)
{
    Frame* true_root = _findTrueRoot(existing_frame);
    for(size_t i=0; i<_kinTrees.size(); ++i)
    {
        if(true_root == _kinTrees[i]->getRootFrame())
        {
            removeChild(_kinTrees[i]);
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

akin::Frame& AkinNode::getFrame(size_t num)
{
    if(num >= _frames.size())
    {
        std::cout << "Requesting an out-of-bounds frame from the AkinNode!"
                  << " -- Requested frame:" << num << ", Total frames:" << _frames.size()-1
                    << std::endl;
        return *_dummyFrame;
    }

    return *_frames[num];
}

akin::Robot& AkinNode::getRobot(size_t num)
{
    if(num >= _robots.size())
    {
        std::cout << "Requesting an out-of-bounds robot from the AkinNode!"
                  << " -- Requested robot:" << num << ", Total robots:" << _robots.size()-1
                     << std::endl;
        return *_dummyRobot;
    }

    return *_robots[num];
}

void AkinCallback::operator ()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::ref_ptr<AkinNode> currentNode =
            dynamic_cast<AkinNode*>(node);

    if(currentNode)
        currentNode->update();

    traverse(node, nv);
}







