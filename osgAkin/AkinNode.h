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

#ifndef AKINNODE_H
#define AKINNODE_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"
#include "KinTree.h"

namespace osgAkin {


class AkinNode : public osg::Group
{
public:

    AkinNode();

    size_t addRootFrame(akin::Frame& new_root_frame);

    size_t addRobot(akin::Robot& new_robot);

    void removeRootFrame(akin::Frame& existing_frame);
    
    virtual void update();

    akin::Frame& getFrame(size_t num);
    akin::Robot& getRobot(size_t num);


protected:

    akin::FramePtrArray _frames;
    akin::RobotPtrArray _robots;

    akin::Frame* _dummyFrame;
    akin::Robot* _dummyRobot;

    akin::Frame* _findTrueRoot(akin::Frame& some_frame);
    
    KinTreePtrArray _kinTrees;

    bool _initialized;
};


class SpinNode : public AkinNode
{
public:

    virtual void update();
};

class KneeNode : public AkinNode
{
public:

    inline KneeNode() : time(0) { }

    virtual void update();

protected:
    double time;
};

} // osgAkin

#endif // AKINNODE_H
