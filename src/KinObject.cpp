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

#include "AkinIncludes.h"

using namespace akin;

KinObject::KinObject(Frame& referenceFrame,
                     std::string myName,
                     verbosity::verbosity_level_t report_level,
                     std::string myType, bool thisIsTheWorld)
{
    if(report_level == verbosity::INHERIT)
        verb.level = referenceFrame.verb.level;
    else
        verb.level = report_level;

    verb.debug() << "Creating '" << myName << "' which is a '" << myType;
    if(!thisIsTheWorld)
        verb << "' in the frame of '" << referenceFrame.name() << "'";
    else
        verb << "'";
    verb.end();

    name(myName);
    _type = myType;

    referenceFrame._gainChildObject(this);
    _referenceFrame = &referenceFrame;
    notifyUpdate();
}

KinObject::KinObject(const KinObject &other)
{
    verb.level = other.verb.level;
    verb.debug() << "Making a copy of object '" << other.name() << "' which is a '" << other.type() << "'"; verb.end();
    _copyValues(other);
}

KinObject& KinObject::operator =(const KinObject& other)
{
    verb.level = other.verb.level;
    verb.debug() << "Assigning object '" << name() << "' to have the values of '" << other.name() << "'";

    _copyValues(other);
}

void KinObject::_copyValues(const KinObject &other)
{
    verb.level = other.verb.level;
    name(other.name());
    _type = other.type();

    other.refFrame()._gainChildObject(this);
    _referenceFrame = &(other.refFrame());
    notifyUpdate();
}

KinObject& KinObject::Generic()
{
    static KinObject generic(Frame::World(), "generic",
                             verbosity::INHERIT, "Placeholder");
    return generic;
}

KinObject::~KinObject()
{
    refFrame()._loseChildObject(this);
}

Frame& KinObject::refFrame() const { return *_referenceFrame; }
std::string KinObject::name() const { return _name; }
void KinObject::name(std::string newName) { _name = newName; }

std::string KinObject::type() const { return _type; }

bool KinObject::changeRefFrame(Frame &newRefFrame)
{
    verb.desc() << "Changing the reference frame of '" << name() << "'' from '"
                << refFrame().name() << "'' to '" << newRefFrame.name() << "'";
    verb.end();

    refFrame()._loseChildObject(this);
    newRefFrame._gainChildObject(this);

    _referenceFrame = &newRefFrame;
    
    return true;
}

bool KinObject::descendsFrom(const Frame &someFrame)
{
    Frame* descentCheck = &refFrame();
    while(!descentCheck->isWorld())
    {
        if(&descentCheck->refFrame() == &someFrame)
            return true;
        descentCheck = &descentCheck->refFrame();
    }

    return false;
}

std::ostream& operator<<(std::ostream& oStrStream, const KinObject& mObject)
{
    oStrStream << mObject.type()
               << " named '" << mObject.name()
               << "' in frame '" << mObject.refFrame().name() << "'";
    return oStrStream;
}

void KinObject::notifyUpdate() { _needsUpdate = true; }
bool KinObject::needsUpdate() { return _needsUpdate; }

void KinObject::_loseParent()
{
    _referenceFrame = &Frame::World();
}
