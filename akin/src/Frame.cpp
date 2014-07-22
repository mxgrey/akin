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

#include "akin/AkinIncludes.h"

using namespace akin;
using namespace std;

Frame::Frame(Frame& referenceFrame, std::string frameName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, frameName, report_level, "Frame"),
    _isWorld(false)
{
    _isFrame = true;
    referenceFrame._gainChildFrame(this);
}

Frame::Frame(const Transform &relativeTf, Frame &referenceFrame, string frameName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, frameName, report_level, "Frame"),
    _isWorld(false),
    _respectToRef(relativeTf)
{
    _isFrame = true;
    referenceFrame._gainChildFrame(this);
}


Frame::Frame(bool createWorld) :
    _isWorld(true),
    KinObject(*this, "World", verbosity::LOG, "World Frame", true)
{
    _isFrame = true;
}

void Frame::_kinitialize(const Frame &other)
{
    _isWorld = false;
    _isFrame = true;
    _respectToRef = other.respectToRef();
    other.refFrame()._gainChildFrame(this);
    notifyUpdate();
}

Frame::~Frame()
{
    for(size_t i=0; i < _registeredObjects.size(); ++i)
        _registeredObjects[i]->_loseParent();
    refFrame()._loseChildFrame(this);
}

Frame& Frame::World()
{
    static Frame world(true);
    return world;
}



void Frame::_gainChildFrame(Frame *child)
{
//    if(_isWorld)
//        return;

    verb.debug() << "Adding Frame '" << child->name() << "' into the Frame '" << name() << "'";
    verb.end();

    _childFrames.push_back(child);
}

void Frame::_loseChildFrame(Frame *child)
{
//    if(_isWorld)
//        return;

    verb.debug() << "Removing '" << child->name() << "' from the Frame '" << name() << "'";
    verb.end();
    int childIndex = -1;
    for(size_t i=0; i<_childFrames.size(); i++)
    {
        if(_childFrames[i] == child)
        {
            childIndex = i;
            break;
        }
    }

    if(childIndex == -1)
    {
        verb.brief() << "Trying to remove frame '" << child->name() << "' from the parentage of '"
                     << name() << "'', that is not its parent!";
        verb.desc() << " Child frames of '" << name() << "' include: ";
        for(int i=0; i<_registeredObjects.size(); i++)
            verb.desc() << " -- " << childFrame(i).name() << "\n";
        verb.end();

        verb.Assert(false, verbosity::ASSERT_CASUAL, "");
    }
    else
    {
        _childFrames.erase(_childFrames.begin()+childIndex);
    }
}

Frame& Frame::childFrame(size_t childFrameNum)
{
//    if(_isWorld)
//    {
//        verb.brief() << "The World Frame does not keep track of its children!";
//        verb.desc() << " Returning the World Frame instead.";
//        verb.end();

//        return World();
//    }

    if(verb.Assert(childFrameNum < _childFrames.size(),
                   verbosity::ASSERT_CASUAL,
                   "Requested non-existent child frame index in Frame '"+name()+"'"))
        return *_childFrames[childFrameNum];
    else
    {
        verb.brief() << "Requested a child of Frame '" << name()
                     << "' which does not exist.";
        verb.desc() << " Returning the World Frame instead.";
        verb.end();
        return Frame::World();
    }
}
size_t Frame::numChildFrames() const { return _childFrames.size(); }

bool Frame::changeRefFrame(Frame &newRefFrame)
{
    if(!verb.Assert(this != &newRefFrame,
                    verbosity::ASSERT_CASUAL,
                    "You requested to make frame '" + name() + "' into its own reference"
                    + " frame, which is not permitted."))
        return false;

    if(!verb.Assert(!newRefFrame.descendsFrom(*this),
                verbosity::ASSERT_CASUAL,
                    "Cannot change the reference frame of '" + name() + "' to '"
                    + newRefFrame.name() + "' because it creates a circular kinematic chain!",
                    " We will leave '" + name() + "' in the frame of '" + refFrame().name() + "'"))
        return false;

    refFrame()._loseChildFrame(this);
    refFrame()._unregisterObject(this);
    
    newRefFrame._gainChildFrame(this);
    newRefFrame._registerObject(this);
    
    _referenceFrame = &newRefFrame;
    
    return true;
}

bool Frame::isWorld() const { return _isWorld; }

void Frame::respectToRef(const Transform &newTf)
{
    if(!verb.Assert(!isWorld(), verbosity::ASSERT_CASUAL,
                    "Cannot change relative transform of the World Frame!",
                    " You have attempted to change the location of the World Frame"
                    " using the respectToRef function, but the World Frame must remain"
                    " a static identity transform"))
    {
        return;
    }

    verb.debug() << "Changing the relative transform going from '"+refFrame().name()
                    +"' to frame '"+name()+"'"; verb.end();

    _respectToRef = newTf;

    notifyUpdate();
}

const Transform& Frame::respectToRef() const { return _respectToRef; }

const Transform& Frame::respectToWorld() const
{
    if(_isWorld)
        return _respectToRef;
    
    if(_needsUpdate)
        _update();

    return _respectToWorld;
}

Transform Frame::withRespectTo(Frame &otherFrame)
{
    return otherFrame.respectToWorld().inverse() * respectToWorld();
}

void Frame::forceUpdate() { _update(); }

void Frame::_update() const
{
    verb.debug() << "Updating frame '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * _respectToRef;

    _needsUpdate = false;
}
