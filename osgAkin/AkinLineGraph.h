#ifndef AKINLINEGRAPH_H
#define AKINLINEGRAPH_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"

namespace osgAkin {

class AkinLineGraph : protected osg::Geometry
{
public:

    ushort addVertex(const akin::Translation& new_vertex, ushort parent_index)
    {
        if(parent_index >= _verts->size())
        {
            std::cout << "You requested to hook a new line vertex onto a non-existent parent index: "
                      << parent_index << std::endl;
            return;
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

    void removeVertex(ushort vertex_index)
    {
        // TODO: What is a meaningful way to make this happen?
    }

    void moveVertex(ushort vertex_index, const akin::Translation& new_location)
    {
        if(vertex_index >= _verts->size())
        {
            std::cout << "You requested to move a non-existent line vertex index: "
                      << vertex_index << std::endl;
            return;
        }
    }

    void updateVertices()
    {
        setVertexArray(_verts);
    }

    void setColor(const osg::Vec4& color)
    {
        (osg::Vec4Array*)& carray = getColorArray();
        (*carray)[0] = color;
        setColorArray(carray);
        setColorBinding(osg::Geometry::BIND_OVERALL);
    }

protected:

    osg::Vec3Array* _verts;

};

} // namespace osgAkin

#endif // AKINLINEGRAPH_H
