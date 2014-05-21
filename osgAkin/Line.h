#ifndef LINE_H
#define LINE_H

#include "LineTree.h"

namespace osgAkin {

class Line : public LineTree
{
public:
    
    inline Line()
    {
        _initialize();
    }
    
    inline Line(const akin::Translation& first_vertex)
    {
        _initialize();
        
        _verts->push_back(osg::Vec3(first_vertex.x(),
                                    first_vertex.y(),
                                    first_vertex.z()));
    }
    
    inline ushort addVertex(const akin::Translation& new_vertex)
    {
        return LineTree::addVertex(new_vertex, _verts->size()-1);
    }
    
    
};

} // namespace osgAkin

#endif // LINE_H
