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

