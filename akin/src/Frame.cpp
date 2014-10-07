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
    _needsAccUpdate = true;
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
    _needsAccUpdate = true;
}


Frame::Frame(bool) :
    KinObject(*this, "World", verbosity::LOG, "World Frame", true),
    _isLink(false),
    _isWorld(true)
{
    _isFrame = true;
    _needsUpdate = false;
    _needsVelUpdate = false;
    _needsAccUpdate = false;
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
    _needsAccUpdate = false;
    notifyAccUpdate();
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
                    "Cannot move the World Frame!",
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

Translation akin::computeLocation(const Translation& ofPoint,
                                  const Frame& inFrame,
                                  const Frame& withRespectToFrame)
{
    if(&inFrame == &withRespectToFrame)
        return ofPoint;
    else if(withRespectToFrame.isWorld())
        return inFrame.respectToWorld()*ofPoint;
    else if(inFrame.isWorld())
        return withRespectToFrame.respectToWorld().inverse()*ofPoint;

    return withRespectToFrame.respectToWorld().inverse() * inFrame.respectToWorld() * ofPoint;
}

const Velocity& Frame::linearVelocity() const
{
    if(_isWorld)
        return _relativeLinearVel;

    if(_needsVelUpdate)
        _velUpdate();

    return _linearVel_wrtWorld;
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

Velocity akin::computeVelocity(const Translation& ofPoint,
                               const Velocity& withVelocity,
                               const Frame& inFrame,
                               const Frame& withRespectToFrame)
{
    if(&inFrame == &withRespectToFrame)
    {
        return withVelocity;
    }
    else if(withRespectToFrame.isWorld())
    {
        // TODO: This has not been tested yet
        return inFrame.linearVelocity()
             + inFrame.respectToWorld().rotation()*withVelocity
             + inFrame.angularVelocity().cross(
                    inFrame.respectToWorld().rotation()*ofPoint);
    }
    else if(inFrame.isWorld())
    {
        return withRespectToFrame.respectToWorld().rotation().transpose()*(
                    withVelocity - withRespectToFrame.linearVelocity()
                    - withRespectToFrame.angularVelocity().cross(
                        ofPoint-withRespectToFrame.respectToWorld().translation()) );
    }

    const Transform& T = inFrame.withRespectTo(withRespectToFrame);

    return inFrame.linearVelocity(withRespectToFrame) + T.rotation()*withVelocity
           +inFrame.angularVelocity(withRespectToFrame).cross(T.rotation()*ofPoint);
}

const Velocity& Frame::relativeLinearVelocity() const
{
    return _relativeLinearVel;
}

void Frame::relativeLinearVelocity(const Velocity &v)
{
    if(!verb.Assert(!isWorld(), verbosity::ASSERT_CASUAL,
                    "Cannot move the World Frame!",
                    " You have attempted to change the relative linear velocity of the"
                    " World Frame using the relativeLinearVelocity(~) function, but"
                    " the World Frame must remain static."))
    {
        return;
    }

    _relativeLinearVel = v;
    notifyVelUpdate();
}

const Velocity& Frame::angularVelocity() const
{
    if(_isWorld)
        return _relativeAngularVel;

    if(_needsVelUpdate)
        _velUpdate();

    return _angularVel_wrtWorld;
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
    return _relativeAngularVel;
}

void Frame::relativeAngularVelocity(const Velocity& w)
{
    if(!verb.Assert(!isWorld(), verbosity::ASSERT_CASUAL,
                    "Cannot move the World Frame!",
                    " You have attempted to change the relative angular velocity of the"
                    " World Frame using the relativeAngularVelocity(~) function, but"
                    " the World Frame must remain static."))
    {
        return;
    }

    _relativeAngularVel = w;
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

void Frame::relativeVelocity(const Velocity &v, coord_t type)
{
    if(LINEAR==type)
        relativeLinearVelocity(v);
    else if(ANGULAR==type)
        relativeAngularVelocity(v);
}

Screw Frame::velocity(const Frame& withRespectToFrame) const
{
    return Screw(linearVelocity(withRespectToFrame), angularVelocity(withRespectToFrame));
}

Screw Frame::relativeVelocity() const
{
    return Screw(relativeLinearVelocity(), relativeAngularVelocity());
}

void Frame::relativeVelocity(const Screw &v_w)
{
    relativeLinearVelocity(v_w.block<3,1>(0,0));
    relativeAngularVelocity(v_w.block<3,1>(3,0));
}

const Acceleration& Frame::linearAcceleration() const
{
    if(_isWorld)
        return _relativeLinearAcc;

    if(_needsAccUpdate)
    {
        _accUpdate();
    }

    return _linearAcc_wrtWorld;
}

Acceleration Frame::linearAcceleration(const Frame &withRespectToFrame) const
{
    if(&withRespectToFrame == &refFrame())
        return relativeLinearAcceleration();
    else if(withRespectToFrame.isWorld())
        return linearAcceleration();

    const Eigen::Vector3d& pr = respectToWorld().translation()
                     - withRespectToFrame.respectToWorld().translation();
    const Eigen::Vector3d& vr = linearVelocity()-withRespectToFrame.linearVelocity()
                     - withRespectToFrame.angularVelocity().cross(pr);

    // TODO: Consider just using computeAcceleration(~) here
    return withRespectToFrame.respectToWorld().rotation().transpose()*
            (linearAcceleration() - withRespectToFrame.linearAcceleration()
             - withRespectToFrame.angularAcceleration().cross(pr)
             - 2*withRespectToFrame.angularVelocity().cross(vr)
             - withRespectToFrame.angularVelocity().cross(
                 withRespectToFrame.angularVelocity().cross(pr)));
}

Acceleration akin::computeAcceleration(const Translation &ofPoint,
                                       const Velocity &withVelocity,
                                       const Acceleration &withAcceleration,
                                       const Frame &inFrame,
                                       const Frame &withRespectToFrame)
{
    if(&inFrame == &withRespectToFrame)
    {
        return withAcceleration;
    }
    else if(withRespectToFrame.isWorld())
    {
        // TODO: This has not been tested yet
        const Eigen::Vector3d& pr = inFrame.respectToWorld()*ofPoint;
        const Eigen::Vector3d& vr = inFrame.respectToWorld().rotation()*withVelocity;

        return inFrame.linearAcceleration() + inFrame.respectToWorld().rotation()*withAcceleration
               + 2*inFrame.angularVelocity().cross(vr)
               + inFrame.angularAcceleration().cross(pr)
               + inFrame.angularVelocity().cross(inFrame.angularVelocity().cross(pr));
    }
    else if(inFrame.isWorld())
    {
        const Eigen::Vector3d& pr = ofPoint - withRespectToFrame.respectToWorld().translation();
        const Eigen::Vector3d& w = withRespectToFrame.angularVelocity();
        const Eigen::Vector3d& vr = withVelocity - withRespectToFrame.linearVelocity() - w.cross(pr);
        const Eigen::Vector3d& alpha = withRespectToFrame.angularAcceleration();

        return withRespectToFrame.respectToWorld().rotation().transpose()*(
                    withAcceleration - withRespectToFrame.linearAcceleration()
                    - 2*w.cross(vr) - alpha.cross(pr) - w.cross(w.cross(pr)));
    }

    const Transform& T = inFrame.withRespectTo(withRespectToFrame);
    const Eigen::Vector3d& pr = T.rotation()*ofPoint;
    const Eigen::Vector3d& vr = T.rotation()*withVelocity;
    const Eigen::Vector3d& w = inFrame.angularVelocity(withRespectToFrame);
    const Eigen::Vector3d& alpha = inFrame.angularAcceleration(withRespectToFrame);

    return inFrame.linearAcceleration(withRespectToFrame) + T.rotation()*withAcceleration
           + alpha.cross(pr) + 2*w.cross(vr) + w.cross(w.cross(pr));
}

const Acceleration& Frame::relativeLinearAcceleration() const
{
    return _relativeLinearAcc;
}

void Frame::relativeLinearAcceleration(const Acceleration& a)
{
    if(!verb.Assert(!isWorld(), verbosity::ASSERT_CASUAL,
                    "Cannot move the World Frame!",
                    " You have attempted to change the relative linear acceleration of the"
                    " World Frame using the relativeLinearAcceleration(~) function, but"
                    " the World Frame must remain static."))
    {
        return;
    }

    _relativeLinearAcc = a;
    notifyAccUpdate();
}

const Acceleration& Frame::angularAcceleration() const
{
    if(_isWorld)
        return _relativeAngularAcc;

    if(_needsAccUpdate)
    {
        _accUpdate();
    }

    return _angularAcc_wrtWorld;
}

Acceleration Frame::angularAcceleration(const Frame &withRespectToFrame) const
{
    if(withRespectToFrame.isWorld())
        return angularAcceleration();
    else if(&withRespectToFrame == &refFrame())
        return relativeAngularAcceleration();

    return withRespectToFrame.respectToWorld().rotation().transpose()*
            (angularAcceleration() - withRespectToFrame.angularAcceleration()
             - withRespectToFrame.angularVelocity().cross(
                 angularVelocity()-withRespectToFrame.angularVelocity()));
}

const Acceleration& Frame::relativeAngularAcceleration() const
{
    return _relativeAngularAcc;
}

void Frame::relativeAngularAcceleration(const Acceleration &w_dot)
{
    if(!verb.Assert(!isWorld(), verbosity::ASSERT_CASUAL,
                    "Cannot move the World Frame!",
                    " You have attempted to change the relative angular acceleration of the"
                    " World Frame using the relativeAngularAcceleration(~) function, but"
                    " the World Frame must remain static."))
    {
        return;
    }

    _relativeAngularAcc = w_dot;
    notifyAccUpdate();
}

const Acceleration& Frame::acceleration(coord_t type) const
{
    if(LINEAR==type)
        return linearAcceleration();
    else if(ANGULAR==type)
        return angularAcceleration();

    return linearAcceleration();
}

Acceleration Frame::acceleration(coord_t type, const Frame &withRespectToFrame) const
{
    if(LINEAR==type)
        return linearAcceleration(withRespectToFrame);
    else if(ANGULAR==type)
        return angularAcceleration(withRespectToFrame);

    return angularAcceleration(withRespectToFrame);
}

void Frame::relativeAcceleration(const Acceleration& a, coord_t type)
{
    if(LINEAR==type)
        relativeLinearAcceleration(a);
    else if(ANGULAR==type)
        relativeAngularAcceleration(a);
}

Screw Frame::acceleration(const Frame &withRespectToFrame) const
{
    return Screw(linearAcceleration(withRespectToFrame), angularAcceleration(withRespectToFrame));
}

Screw Frame::relativeAcceleration() const
{
    return Screw(relativeLinearAcceleration(), relativeAngularAcceleration());
}

void Frame::relativeAcceleration(const Screw& a_wdot)
{
    relativeLinearAcceleration(a_wdot.block<3,1>(0,0));
    relativeAngularAcceleration(a_wdot.block<3,1>(3,0));
}

void Frame::notifyUpdate()
{
    notifyVelUpdate();
    KinObject::notifyUpdate();

    if(isWorld())
    {
        _needsUpdate = false;
        _needsVelUpdate = false;
        _needsAccUpdate = false;
    }
}

void Frame::demandPoseUpdate() const { _update(); }

void Frame::notifyVelUpdate()
{
    notifyAccUpdate();
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

void Frame::notifyAccUpdate()
{
    if(_needsAccUpdate)
        return;

    _needsAccUpdate = true;
    for(size_t i=0; i<numChildFrames(); ++i)
        childFrame(i).notifyAccUpdate();
}

bool Frame::needsAccUpdate() const { return _needsAccUpdate; }

void Frame::demandAccUpdate() const { _accUpdate(); }

void Frame::_update() const
{
    verb.debug() << "Updating frame '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * _respectToRef;

    _needsUpdate = false;
}

void Frame::_velUpdate() const
{
    verb.debug() << "Updating velocity of frame '"+name()+"'"; verb.end();

    _linearVel_wrtWorld = refFrame().linearVelocity()
                        + refFrame().respectToWorld().rotation()*_relativeLinearVel
            + refFrame().angularVelocity().cross(
                            refFrame().respectToWorld().rotation()*_respectToRef.translation());

    _angularVel_wrtWorld = refFrame().angularVelocity()
                         + refFrame().respectToWorld().rotation()*_relativeAngularVel;

    _needsVelUpdate = false;
}

void Frame::_accUpdate() const
{
    verb.debug() << "Updating acceleration of frame '"+name()+"'"; verb.end();

    Eigen::Vector3d pr = refFrame().respectToWorld().rotation()*_respectToRef.translation();

    _linearAcc_wrtWorld = refFrame().linearAcceleration()
                        + refFrame().respectToWorld().rotation()*_relativeLinearAcc
                        + refFrame().angularAcceleration().cross(pr)
            + 2*refFrame().angularVelocity().cross(
                    refFrame().respectToWorld().rotation()*_relativeLinearVel)
            + refFrame().angularVelocity().cross(refFrame().angularVelocity().cross(pr));

    _angularAcc_wrtWorld = refFrame().angularAcceleration()
                         + refFrame().respectToWorld().rotation()*_relativeAngularAcc
            + refFrame().angularVelocity().cross(
                    refFrame().respectToWorld().rotation()*_relativeAngularVel);

    _needsAccUpdate = false;
}

std::ostream& operator<<(std::ostream& oStrStream, const akin::Frame& mFrame)
{
    oStrStream << (akin::KinObject&)mFrame << " has relative transform:\n" << mFrame.respectToRef()
               << "\nAnd World transform:\n" << mFrame.respectToWorld() << std::endl;
    return oStrStream;
}
