#ifndef AKINLINEGRAPH_H
#define AKINLINEGRAPH_H

#include "AkinIncludes.h"
#include "IncludeOSG.h"

namespace osgAkin {

class AkinLineTree : protected osg::Geometry
{
public:
    
    inline AkinLineTree(const akin::Translation& first_vertex = (akin::Translation)akin::Translation::Zero())
    {
        _verts->push_back(osg::Vec3(first_vertex.x(),
                                    first_vertex.y(),
                                    first_vertex.z()));
    }

    inline ushort addVertex(const akin::Translation& new_vertex, ushort parent_index)
    {
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

    inline void removeVertex(ushort vertex_index)
    {
        // TODO: What is a meaningful way to make this happen?
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
    }

    inline void updateVertices()
    {
        setVertexArray(_verts);
    }

    inline void setColor(const osg::Vec4& color)
    {
        osg::ref_ptr<osg::Vec4Array> carray = dynamic_cast<osg::Vec4Array*>(getColorArray());
        if(carray)
        {
            (*carray)[0] = color;
            setColorArray(carray);
            setColorBinding(osg::Geometry::BIND_OVERALL);
        }
        else
        {
            std::cout << "Your line graph color was not set up with a Vec4Array. Something weird is going on" << std::endl;
        }
    }

protected:

    osg::Vec3Array* _verts;

};

typedef std::vector<AkinLineTree*> AkinLineTreePtrArray;

} // namespace osgAkin

#endif // AKINLINEGRAPH_H
