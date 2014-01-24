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

#ifndef OSGAKINLINETREE_H
#define OSGAKINLINETREE_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"

namespace osgAkin {

class LineTree : public osg::Geometry
{
public:

    inline LineTree()
    {
        _initialize();
    }
    
    inline LineTree(const akin::Translation& first_vertex)
    {
        _initialize();

        _verts->push_back(osg::Vec3(first_vertex.x(),
                                    first_vertex.y(),
                                    first_vertex.z()));
    }

    inline ushort addVertex(const akin::Translation& new_vertex, ushort parent_index)
    {
        if(_verts->size()==0)
        {
            _verts->push_back(osg::Vec3(new_vertex.x(),
                                        new_vertex.y(),
                                        new_vertex.z()));
            return 0;
        }

        if(parent_index >= _verts->size())
        {
            std::cout << "You requested to hook a new line vertex onto a non-existent parent index: "
                      << parent_index << std::endl;
            return -1;
        }

        _verts->push_back(osg::Vec3(new_vertex.x(),
                                    new_vertex.y(),
                                    new_vertex.z()));
        osg::DrawElementsUShort* new_elem = new osg::DrawElementsUShort(osg::PrimitiveSet::LINES,0);
        new_elem->push_back(parent_index);
        new_elem->push_back(_verts->size()-1);
        addPrimitiveSet(new_elem);
        return _verts->size()-1;
    }

//    inline void removeVertex(ushort vertex_index)
//    {
//         //TODO: What is a meaningful way to make this happen?
//    }

    inline ushort vertexCount()
    {
        return _verts->size();
    }
    
    inline void clear()
    {
        _verts->resize(0);
    }

    inline void moveVertex(ushort vertex_index, const akin::Translation& new_location)
    {
        if(vertex_index >= _verts->size())
        {
            std::cout << "You requested to move a non-existent line vertex index: "
                      << vertex_index << std::endl;
            return;
        }
        
        osg::Vec3& vert = (*_verts)[vertex_index];
        vert.x() = new_location.x();
        vert.y() = new_location.y();
        vert.z() = new_location.z();
    }

    inline akin::Translation getVertex(ushort vertex_index)
    {
        if(vertex_index >= _verts->size())
            return akin::Translation::Zero();

        osg::Vec3& vert = (*_verts)[vertex_index];
        return akin::Translation(vert.x(),
                                 vert.y(),
                                 vert.z());
    }

    inline void updateVertices()
    {
        setVertexArray(_verts);
    }

    inline void setColor(const osg::Vec4& color)
    {
        (*_color)[0] = color;
        setColorArray(_color);
        setColorBinding(osg::Geometry::BIND_OVERALL);
    }

protected:

    inline virtual void _initialize()
    {
        setDataVariance(osg::Object::DYNAMIC);

        _verts = new osg::Vec3Array;
        _color = new osg::Vec4Array;
        _color->resize(1);
        setColor(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
    }

    osg::Vec3Array* _verts;
    osg::Vec4Array* _color;

};

typedef std::vector<LineTree*> LineTreePtrArray;

} // namespace osgAkin

#endif // OSGAKINLINEGRAPH_H
