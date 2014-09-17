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
    _isLink(false),
    _isWorld(false)
{
    _isFrame = true;
    referenceFrame._gainChildFrame(this);
    _needsVelUpdate = true;
}

Frame::Frame(const Transform &relativeTf, Frame &referenceFrame, string frameName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, frameName, report_level, "Frame"),
    _respectToRef(relativeTf),
    _isLink(false),
    _isWorld(false)
{
    _isFrame = true;
    referenceFrame._gainChildFrame(this);
    _needsVelUpdate = true;
}


Frame::Frame(bool) :
    KinObject(*this, "World", verbosity::LOG, "World Frame", true),
    _isLink(false),
    _isWorld(true)
{
    _isFrame = true;
}

void Frame::_kinitialize(const Frame &other)
{
    _isWorld = false;
    _isFrame = true;
    _respectToRef = other.respectToRef();
    other.refFrame()._gainChildFrame(this);
    _needsUpdate = false;
    notifyUpdate();
    _needsVelUpdate = false;
    notifyVelUpdate();
}

Frame::~Frame()
{
    for(size_t i=0; i < _registeredObjects.size(); ++i)
        _registeredObjects[i]->_loseParent(this);
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
        for(size_t i=0; i<_registeredObjects.size(); i++)
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

bool Frame::isLink() const { return _isLink; }

void Frame::respectToRef(const Transform &newTf)
{
    if(!verb.Assert(!isWorld(), verbosity::ASSERT_CASUAL,
                    "Cannot change relative transform of the World Frame!",
                    " You have attempted to change the location of the World Frame"
                    " using the respectToRef(~) function, but the World Frame must remain"
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

Transform Frame::withRespectTo(const Frame &otherFrame) const
{
    if(otherFrame.isWorld())
        return respectToWorld();
    else if(&otherFrame == &refFrame())
        return respectToRef();

    return otherFrame.respectToWorld().inverse() * respectToWorld();
}

const Velocity& Frame::linearVelocity() const
{
    if(_isWorld)
        return _relativeLinearV;

    if(_needsUpdate || _needsVelUpdate)
        _velUpdate();

    return _linearV_wrtWorld;
}

Velocity Frame::linearVelocity(const Frame& withRespectToFrame) const
{
    if(withRespectToFrame.isWorld())
        return linearVelocity();
    else if(&withRespectToFrame == &refFrame())
        return relativeLinearVelocity();

    return withRespectToFrame.respectToWorld().rotation().transpose()*
            (linearVelocity() - withRespectToFrame.linearVelocity()
            - withRespectToFrame.angularVelocity().cross(
                respectToWorld().translation()
                - withRespectToFrame.respectToWorld().translation()));
}

const Velocity& Frame::relativeLinearVelocity() const
{
    return _relativeLinearV;
}

void Frame::relativeLinearVelocity(const Velocity &v)
{
    if(!verb.Assert(!isWorld(), verbosity::ASSERT_CASUAL,
                    "Cannot change the velocity of the World Frame!",
                    " You have attempted to change the relative linear velocity of the"
                    " World Frame using the relativeLinearVelocity(~) function, but"
                    " the World Frame must remain static."))
    {
        return;
    }

    _relativeLinearV = v;
    notifyVelUpdate();
}

const Velocity& Frame::angularVelocity() const
{
    if(_isWorld)
        return _relativeAngularV;

    if(_needsUpdate || _needsVelUpdate)
        _velUpdate();

    return _angularV_wrtWorld;
}

Velocity Frame::angularVelocity(const Frame &withRespectToFrame) const
{
    if(withRespectToFrame.isWorld())
        return angularVelocity();
    else if(&withRespectToFrame == &refFrame())
        return relativeAngularVelocity();

    return withRespectToFrame.respectToWorld().rotation().transpose()*
            (angularVelocity() - withRespectToFrame.angularVelocity());
}

const Velocity& Frame::relativeAngularVelocity() const
{
    return _relativeAngularV;
}

void Frame::relativeAngularVelocity(const Velocity& w)
{
    if(!verb.Assert(!isWorld(), verbosity::ASSERT_CASUAL,
                    "Cannot change the velocity of the World Frame!",
                    " You have attempted to change the relative angular velocity of the"
                    " World Frame using the relativeAngularVelocity(~) function, but"
                    " the World Frame must remain static."))
    {
        return;
    }

    _relativeAngularV = w;
    notifyVelUpdate();
}

const Velocity& Frame::velocity(coord_t type) const
{
    if(LINEAR==type)
        return linearVelocity();
    else if(ANGULAR==type)
        return angularVelocity();

    return linearVelocity();
}

Velocity Frame::velocity(coord_t type, const Frame &withRespectToFrame) const
{
    if(LINEAR==type)
        return linearVelocity(withRespectToFrame);
    else if(ANGULAR==type)
        return angularVelocity(withRespectToFrame);

    return linearVelocity(withRespectToFrame);
}

void Frame::notifyUpdate()
{
    KinObject::notifyUpdate();
    notifyVelUpdate();
}

void Frame::demandPoseUpdate() const { _update(); }

void Frame::notifyVelUpdate()
{
    if(_needsVelUpdate)
    {
        return;
    }

    _needsVelUpdate = true;
    for(size_t i=0; i<numChildFrames(); ++i)
        childFrame(i).notifyVelUpdate();
}

bool Frame::needsVelUpdate() const { return _needsVelUpdate; }

void Frame::demandVelUpdate() const { _velUpdate(); }

void Frame::_update() const
{
    verb.debug() << "Updating frame '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * _respectToRef;

    _needsUpdate = false;
}

void Frame::_velUpdate() const
{
    verb.debug() << "Updating velocity of frame '"+name()+"'"; verb.end();

    _linearV_wrtWorld = refFrame().linearVelocity() 
                        + refFrame().respectToWorld().rotation()*_relativeLinearV
            + refFrame().angularVelocity().cross(
                            refFrame().respectToWorld().rotation()*_respectToRef.translation());

    _angularV_wrtWorld = refFrame().angularVelocity() 
                         + refFrame().respectToWorld().rotation()*_relativeAngularV;

    _needsVelUpdate = false;
}

std::ostream& operator<<(std::ostream& oStrStream, const akin::Frame& mFrame)
{
    oStrStream << (akin::KinObject&)mFrame << " has relative transform:\n" << mFrame.respectToRef()
               << "\nAnd World transform:\n" << mFrame.respectToWorld() << std::endl;
    return oStrStream;
}
