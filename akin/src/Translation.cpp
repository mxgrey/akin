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


using namespace std;
using namespace akin;

const Translation& KinTranslation::respectToWorld() const
{
    if(_needsUpdate)
        _update();

    return _respectToWorld;
}

Translation KinTranslation::withRespectTo(const Frame &someFrame) const
{
    return someFrame.respectToWorld().inverse() * respectToWorld();
}

void KinTranslation::_update() const
{
    verb.debug() << "Updating translation '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * (Translation&)(*this);

    _needsUpdate = false;
}

const FreeVector& KinFreeVector::respectToWorld() const
{
    if(_needsUpdate)
        _update();

    return _respectToWorld;
}

FreeVector KinFreeVector::withRespectTo(const Frame &someFrame) const
{
    // TODO: Investigate if it is okay to leave off Transform()
    return Transform(someFrame.respectToWorld().inverse()) * respectToWorld();
}

void KinFreeVector::_update() const
{
    verb.debug() << "Updating FreeVector '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * (FreeVector&)(*this);

    _needsUpdate = false;
}

const Axis& KinAxis::respectToWorld() const
{
    if(_needsUpdate)
        _update();

    return _respectToWorld;
}

Axis KinAxis::withRespectTo(const Frame &someFrame) const
{
    return Transform(someFrame.respectToWorld().inverse()) * respectToWorld();
}

void KinAxis::_update() const
{
    verb.debug() << "Updating Axis '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * (Axis&)(*this);

    _needsUpdate = false;
}
