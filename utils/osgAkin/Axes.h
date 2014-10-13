#ifndef AXES_H
#define AXES_H

#include "akin/AkinIncludes.h"
#include "osgAkin/IncludeOSG.h"

namespace osgAkin {

class Axes : public osg::Geometry
{
public:
    
    inline Axes(float scale = 1)
    {
        _verts = new osg::Vec3Array;
        _colors = new osg::Vec4Array;
        _verts->resize(5);
        
        _setElements();
        setScale(scale);
    }
    
    inline void setScale(float new_scale)
    {
        (*_verts)[0] = osg::Vec3(0,0,0);
        (*_verts)[1] = osg::Vec3(new_scale,0,0);
        (*_verts)[2] = osg::Vec3(0,new_scale,0);
        (*_verts)[3] = osg::Vec3(0,0,new_scale);
        (*_verts)[4] = osg::Vec3(new_scale,new_scale,0);
        
        setVertexArray(_verts);
    }
    
    
protected:
    
    inline void _setElements()
    {
        osg::DrawElementsUShort* x_elem =
                new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 0);
        x_elem->push_back(0); x_elem->push_back(1);
        addPrimitiveSet(x_elem);
        _colors->push_back(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
        
        osg::DrawElementsUShort* y_elem =
                new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 0);
        y_elem->push_back(0); y_elem->push_back(2);
        addPrimitiveSet(y_elem);
        _colors->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
        
        osg::DrawElementsUShort* z_elem =
                new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 0);
        z_elem->push_back(0); z_elem->push_back(3);
        addPrimitiveSet(z_elem);
        _colors->push_back(osg::Vec4(0.0f,0.0f,1.0f,1.0f));

        osg::DrawElementsUShort* plane_top =
                new osg::DrawElementsUShort(osg::PrimitiveSet::QUADS, 0);
        plane_top->push_back(0); plane_top->push_back(1);
        plane_top->push_back(4); plane_top->push_back(2);
        addPrimitiveSet(plane_top);
        _colors->push_back(osg::Vec4(0.6f,0.6f,1.0f,0.5));

        osg::DrawElementsUShort* plane_bottom =
                new osg::DrawElementsUShort(osg::PrimitiveSet::QUADS, 0);
        plane_bottom->push_back(0); plane_bottom->push_back(2);
        plane_bottom->push_back(4); plane_bottom->push_back(1);
        addPrimitiveSet(plane_bottom);
        _colors->push_back(osg::Vec4(1.0f,1.0f,0.6f,0.5f));
        
        setColorArray(_colors);
        setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    }
    
    osg::Vec3Array* _verts;
    osg::Vec4Array* _colors;
    
};

} // namespace osgAkin


#endif // AXES_H
