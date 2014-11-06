
#include "osgAkin/AkinVisual.h"
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include "osgAkin/Axes.h"

#include <osg/Material>
#include <osg/ColorMatrix>

using namespace akin;
using namespace osgAkin;
using namespace std;



AkinVisual::AkinVisual(const Geometry &visual)
{
    _colors = new osg::Vec4Array;
    _lineWidth = new osg::LineWidth(2.0f);
    _cull = new osg::CullFace(osg::CullFace::BACK);
    _initializeVisual(visual);
}


void AkinVisual::_initializeVisual(const Geometry &visual)
{
    setMatrix(cosg(visual.relative_pose));
    _colors->clear();

    if(Geometry::MESH_FILE == visual.type)
    {
        osg::ref_ptr<Node> file_node = osgDB::readNodeFile(visual.mesh_filename);
        if(!file_node)
        {
            std::cerr << "Could not load file named '"
                      << visual.mesh_filename << "'" << std::endl;
        }
        else
        {
            file_node->getOrCreateStateSet()->setGlobalDefaults();
            addChild(file_node);
        }
    }
    else if(Geometry::BOX == visual.type)
    {
        _makeBox(visual);
    }
    else if(Geometry::AXES == visual.type)
    {
        _makeAxes(visual);
    }
    else if(Geometry::CYLINDER == visual.type)
    {
        std::cout << "Making cylinder" << std::endl;
        _makeCylinder(visual);
    }
}


//osg::Geode* AkinVisual::_makeSphere(const Geometry &visual)
//{
    // TODO: Come up with a decent way of rendering a primitive sphere
    
//    osg::Sphere* sphere = new osg::Sphere(osg::Vec3(visual.relative_pose.translation()[0],
//                                                    visual.relative_pose.translation()[1],
//                                                    visual.relative_pose.translation()[2]),
//                                            visual.scale[0]);
//    osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere);
//    osg::Vec4 originalColor = sphereDrawable->getColor();
//    sphereDrawable->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
    
//    osg::ref_ptr<osg::Geode> sphere_node = new osg::Geode;
//    sphere_node->addDrawable(sphereDrawable);
    
//    addChild(sphere_node);
    
//    sphereDrawable->setColor(originalColor);
    
    
    
//    return sphere_node;
//}

osg::Geode* AkinVisual::_makeAxes(const Geometry &visual)
{
    osg::ref_ptr<osg::Geometry> axesGeom = new osgAkin::Axes(visual.scale[0]);

    osg::ref_ptr<osg::Geode> axes_node = new osg::Geode;
    _setGeodeModes(axes_node);
    axes_node->addDrawable(axesGeom);

    addChild(axes_node);

    return axes_node;
}

// TODO: Split out the vertices to be per-primitive and then add normal array data
osg::Geode* AkinVisual::_makeBox(const Geometry &visual)
{
    osg::ref_ptr<osg::Geometry> boxGeom = new osg::Geometry;

    osg::ref_ptr<osg::Vec3Array> boxVerts = new osg::Vec3Array;

    double v[2] = { -0.5, 0.5 };
    for(int i=0; i<2; ++i)
    {
        double x = v[i]*visual.scale[0];
        for(int j=0; j<2; ++j)
        {
            double y = v[j]*visual.scale[1];
            for(int k=0; k<2; ++k)
            {
                double z = v[k]*visual.scale[2];
                boxVerts->push_back(osg::Vec3(x,y,z));
            }
        }
    }

    boxGeom->setVertexArray(boxVerts);

    osg::ref_ptr<osg::DrawElementsUShort> faces = new osg::DrawElementsUShort(osg::PrimitiveSet::QUADS, 0);
    faces->push_back(0); faces->push_back(1); faces->push_back(3); faces->push_back(2); // back
    faces->push_back(4); faces->push_back(6); faces->push_back(7); faces->push_back(5); // front
    faces->push_back(1); faces->push_back(5); faces->push_back(7); faces->push_back(3); // top
    faces->push_back(0); faces->push_back(2); faces->push_back(6); faces->push_back(4); // bottom
    faces->push_back(1); faces->push_back(0); faces->push_back(4); faces->push_back(5); // left
    faces->push_back(3); faces->push_back(7); faces->push_back(6); faces->push_back(2); // right

    osg::ref_ptr<osg::DrawElementsUShort> lines = new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 0);
    for(size_t i=0; i<faces->size(); ++i)
        lines->push_back((*faces)[i]);
//    lines->push_back(2); lines->push_back(0);
//    lines->push_back(5); lines->push_back(4);
//    lines->push_back(3); lines->push_back(1);
//    lines->push_back(4); lines->push_back(0);


    boxGeom->addPrimitiveSet(faces);
    boxGeom->addPrimitiveSet(lines);

    if(visual.colors.size()>0)
        _colors->push_back(cosg(visual.colors[0]));
    else
        _colors->push_back(cosg(akin::ColorSpec::Gray()));

    if(visual.colors.size()>1)
        _colors->push_back(cosg(visual.colors[1]));
    else
        _colors->push_back(cosg(akin::ColorSpec::Black()));

    boxGeom->setColorArray(_colors);
    boxGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

    osg::ref_ptr<osg::Geode> box_node = new osg::Geode;
    _setGeodeModes(box_node);
    box_node->addDrawable(boxGeom);

    addChild(box_node);
    
    return box_node;
}

osg::Geode* AkinVisual::_makeCylinder(const Geometry &visual)
{
    osg::ref_ptr<osg::Geometry> cylGeom = new osg::Geometry;

    osg::ref_ptr<osg::Vec3Array> cylVerts = new osg::Vec3Array;

    double xe = visual.scale[0];
    double ye = visual.scale[1];
    double h = visual.scale[2];

    size_t res = 100;
    cylVerts->reserve(2*res);
    for(size_t i=0; i<res; ++i)
    {
        double theta = (double)(i)/(double)(res)*2*M_PI;
        double x = xe*cos(theta);
        double y = ye*sin(theta);

        for(size_t k=0; k<2; ++k)
        {
            double z = k==0? -h/2 : h/2;
            cylVerts->push_back(osg::Vec3(x,y,z));
        }
    }

    cylVerts->push_back(osg::Vec3(0,0,-h/2));
    cylVerts->push_back(osg::Vec3(0,0, h/2));

    cylGeom->setVertexArray(cylVerts);

    osg::ref_ptr<osg::DrawElementsUShort> faces = new osg::DrawElementsUShort(osg::PrimitiveSet::QUADS,0);
    faces->reserve(res);

    for(size_t i=0; i<res; ++i)
    {
        faces->push_back(2*i+1);
        faces->push_back(2*i);
        if(i<res-1)
        {
            faces->push_back(2*i+2);
            faces->push_back(2*i+3);
        }
        else
        {
            faces->push_back(0);
            faces->push_back(1);
        }
    }

    osg::ref_ptr<osg::DrawElementsUShort> cap = new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLES,0);
    cap->reserve(2*res);
    for(size_t i=0; i<res; ++i)
    {
        if(i < res-1)
            cap->push_back(2*i+2);
        else
            cap->push_back(0);
        cap->push_back(2*i);
        cap->push_back(res);

        cap->push_back(2*i+1);
        if(i < res-1)
            cap->push_back(2*i+3);
        else
            cap->push_back(1);
        cap->push_back(res+1);
    }

    osg::ref_ptr<osg::DrawElementsUShort> lines = new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 0);
    for(size_t i=0; i<res; ++i)
    {
        lines->push_back(2*i);
        lines->push_back(2*i+1);

        lines->push_back(2*i);
        if(i<res-1)
            lines->push_back(2*i+2);
        else
            lines->push_back(0);

        lines->push_back(2*i+1);
        if(i<res-1)
            lines->push_back(2*i+3);
        else
            lines->push_back(1);
    }

    cylGeom->addPrimitiveSet(faces);
    cylGeom->addPrimitiveSet(cap);
    cylGeom->addPrimitiveSet(lines);

    if(visual.colors.size() > 0)
        _colors->push_back(cosg(visual.colors[0]));
    else
        _colors->push_back(cosg(akin::ColorSpec::Gray()));
    _colors->push_back(_colors->back());


    if(visual.colors.size()>1)
        _colors->push_back(cosg(visual.colors[1]));
    else
        _colors->push_back(cosg(akin::ColorSpec::Black()));

    cylGeom->setColorArray(_colors);
    cylGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

    osg::ref_ptr<osg::Geode> cyl_node = new osg::Geode;
    _setGeodeModes(cyl_node);
    cyl_node->addDrawable(cylGeom);

    addChild(cyl_node);

    return cyl_node;
}

void AkinVisual::_setGeodeModes(osg::Geode *geode)
{
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setAttributeAndModes(_lineWidth);
    geode->getOrCreateStateSet()->setAttributeAndModes(_cull, osg::StateAttribute::ON);
    geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
}
