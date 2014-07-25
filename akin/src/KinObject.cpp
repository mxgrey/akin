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

KinObject::KinObject(Frame& referenceFrame,
                     const std::string& myName,
                     verbosity::verbosity_level_t report_level,
                     const std::string& myType, bool thisIsTheWorld)
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

    _isFrame = false;
    _visualsUpdate = false;

    referenceFrame._registerObject(this);
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
    verb.debug() << "Assigning object '" << name() << "' to have the values of '" << other.name() << "'"; verb.end();

    _copyValues(other);
    
    return *this;
}

void KinObject::_copyValues(const KinObject &other)
{
    verb.level = other.verb.level;
    name(other.name());
    _type = other.type();

    other.refFrame()._registerObject(this);
    _referenceFrame = &(other.refFrame());

    _isFrame = other.isFrame();

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
    refFrame()._unregisterObject(this);
}

Frame& KinObject::refFrame() const { return *_referenceFrame; }
const Frame& KinObject::const_refFrame() const { return *_referenceFrame; }
const std::string& KinObject::name() const { return _name; }
bool KinObject::name(const std::string& newName) { _name = newName; return true; }

const std::string& KinObject::type() const { return _type; }

bool KinObject::changeRefFrame(Frame &newRefFrame)
{
    verb.desc() << "Changing the reference frame of '" << name() << "'' from '"
                << refFrame().name() << "'' to '" << newRefFrame.name() << "'";
    verb.end();

    refFrame()._unregisterObject(this);
    newRefFrame._registerObject(this);

    _referenceFrame = &newRefFrame;
    
    return true;
}

void KinObject::_registerObject(KinObject *child)
{
    verb.debug() << "Adding object '" << child->name() << "' to the Frame '" << name() << "'";
    verb.end();

    _registeredObjects.push_back(child);

    child->notifyUpdate();
}

void KinObject::_unregisterObject(KinObject *child)
{
    verb.debug() << "Removing '" << child->name() << "' as an object of the Frame '" << name() << "'";
    verb.end();

    int childIndex = -1;
    for(size_t i=0; i<_registeredObjects.size(); ++i)
    {
        if(_registeredObjects[i] == child)
        {
            childIndex = i;
            break;
        }
    }

    if(childIndex == -1)
    {
        verb.brief() << "Trying to remove '" << child->name() << "' from the parentage of '"
                     << name() << "', but they are not related!";
        verb.desc() << " Children of '" << name() << "' include: ";
        for(size_t i=0; i<_registeredObjects.size(); ++i)
            verb.desc() << " -- " << registeredObject(i).name() << "\n";
        verb.end();

        verb.Assert(false, verbosity::ASSERT_CASUAL, "");
    }
    else
    {
        _registeredObjects.erase(_registeredObjects.begin()+childIndex);
    }
}

KinObject& KinObject::registeredObject(size_t objNum)
{
    if(verb.Assert(objNum < _registeredObjects.size(),
                   verbosity::ASSERT_CASUAL,
                   "Requested non-existent child object index in Frame '"+name()+"'"))
        return *_registeredObjects[objNum];
    else
    {
        verb.brief() << "Requested a child object of Frame '" << name()
                     << "' which does not exist."
                     << " Returning a generic object instead.";
        verb.end();
        return KinObject::Generic();
    }
}
size_t KinObject::numRegisteredObjects() const { return _registeredObjects.size(); }

bool KinObject::descendsFrom(const Frame &someFrame) const
{
    if(&const_refFrame() == &someFrame)
        return true;
    
    const Frame* descentCheck = &const_refFrame();
    while(!descentCheck->isWorld())
    {
        if(&descentCheck->const_refFrame() == &someFrame)
            return true;
        descentCheck = &descentCheck->const_refFrame();
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

void KinObject::notifyUpdate()
{
    verb.debug() << "Instructing KinObject '"+name()+"' to update"; verb.end();

    if(_needsUpdate)
    {
        verb.debug() << "KinObject '" + name() + "' already knows that it needs to update!"; verb.end();
        return;
    }
    
    _needsUpdate = true;
    for(size_t i=0; i<numRegisteredObjects(); ++i)
    {
        registeredObject(i).notifyUpdate();
    }
}

bool KinObject::needsUpdate() { return _needsUpdate; }

void KinObject::_loseParent(akin::KinObject*)
{
    _referenceFrame = &Frame::World();
}


size_t KinObject::addVisual(const Geometry &visual_geometry)
{
    _visualsUpdate = true;
    _visuals.push_back(visual_geometry);
    return _visuals.size()-1;
}

bool KinObject::removeVisual(size_t num)
{
    if(num >= _visuals.size())
        return false;
    

    _visualsUpdate = true;
    _visuals.erase(_visuals.begin()+num);
    return true;
}

void KinObject::clearVisuals()
{
    _visualsUpdate = true;
    _visuals.clear();
}

const Geometry& KinObject::peekVisual(size_t num) const
{
    if( num >= _visuals.size() )
        return Geometry::Empty();
    
    return _visuals[num];
}

const GeometryArray& KinObject::peekVisuals() const
{
    return _visuals;
}

size_t KinObject::addCollider(const Geometry &colliding_geometry)
{
    _collidersUpdate = true;
    _colliders.push_back(colliding_geometry);
    return _colliders.size()-1;
}

bool KinObject::removeCollider(size_t num)
{
    if(num >= _colliders.size())
        return false;
    
    _collidersUpdate = true;
    _colliders.erase(_colliders.begin()+num);
    return true;
}

void KinObject::clearColliders()
{
    _collidersUpdate = true;
    _colliders.clear();
}

const Geometry& KinObject::peekCollider(size_t num) const
{
    if(num >= _colliders.size())
        return Geometry::Empty();
    
    return _colliders[num];
}

const GeometryArray& KinObject::peekColliders() const
{
    return _colliders;
}

bool KinObject::visualsChanged() const
{
    return _visualsUpdate;
}

bool KinObject::collidersChanged() const
{
    return _collidersUpdate;
}

const GeometryArray& KinObject::grabVisualsAndReset()
{
    _visualsUpdate = false;
    return _visuals;
}

const GeometryArray& KinObject::grabCollidersAndReset()
{
    _collidersUpdate = false;
    return _colliders;
}
