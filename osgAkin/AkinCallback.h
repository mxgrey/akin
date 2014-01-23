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

#ifndef AKINCALLBACK_H
#define AKINCALLBACK_H

#include "AkinIncludes.h"
#include "AkinData.h"
#include "AkinNode.h"
#include "osg/NodeCallback"
#include "osg/NodeVisitor"

namespace osgAkin {

class AkinCallback : public osg::NodeCallback
{
public:

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osg::ref_ptr<AkinNode> currentNode =
                dynamic_cast<AkinNode*>(node);
        
        currentNode->update();

        traverse(node, nv);
    }
};

class SpinData : public osg::Referenced
{
public:
    osg::Vec3Array* lineVerts;
    osg::Geometry* geom;
};

class SpinCallback : public osg::NodeCallback
{
public:

    SpinCallback() : time(0) { }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
//        osg::ref_ptr<osg::Vec3Array> lineVerts =
//                dynamic_cast<osg::Vec3Array*>(node->getUserData());
        osg::ref_ptr<SpinData> data =
                dynamic_cast<SpinData*>(node->getUserData());
        
        if(data)
        {
//            std::cout << "We have received line verts: " << (*lineVerts)[1].x() << " " << (*lineVerts)[1].z() << std::endl;
//            float r = sqrt((*lineVerts)[1].x() * (*lineVerts)[1].x() + (*lineVerts)[1].z() * (*lineVerts)[1].z());
//            time += 0.05;
//            (*lineVerts)[1].x() = r * cos(time);
//            (*lineVerts)[1].z() = r * sin(time);
            osg::Vec3& vec = (*data->lineVerts)[1];
//            std::cout << "We have received line verts: " << vec.x() << ", " << vec.z() << std::endl;
            float r = sqrt(vec.x()*vec.x() + vec.z()*vec.z());
            time += 0.05;
            vec.x() = r * cos(time);
            vec.z() = r * sin(time);
            
            data->geom->setVertexArray(data->lineVerts);
        }

        traverse(node, nv);
    }

protected:
    float time;
};


} // namespace osgAkin

#endif // AKINCALLBACK_H
