
#include "AkinVisual.h"
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/Geometry>

using namespace akin;
using namespace osgAkin;
using namespace std;


AkinVisual::AkinVisual(const Geometry &visual)
{
    _colors = new osg::Vec4Array;
    _initializeVisual(visual);
}


void AkinVisual::_initializeVisual(const Geometry &visual)
{
    setMatrix(cosg(visual.relative_pose));
    _colors->clear();

    if(visual.type == Geometry::MESH_FILE)
    {
        osg::ref_ptr<Node> file_node = osgDB::readNodeFile(visual.mesh_filename);
        if(!file_node)
        {
            std::cerr << "Could not load file named '"
                      << visual.mesh_filename << "'" << std::endl;
        }
        else
        {
            addChild(file_node);
        }
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
            double y = v[i]*visual.scale[1];
            for(int k=0; k<2; ++k)
            {
                double z = v[i]*visual.scale[2];
                boxVerts->push_back(osg::Vec3(x,y,z));
            }
        }
    }

    boxGeom->setVertexArray(boxVerts);

    for(int i=0; i<6; ++i)
    {
        osg::ref_ptr<osg::DrawElementsUShort> boxFace =
                new osg::DrawElementsUShort(osg::PrimitiveSet::QUADS, 0);
        for(int j=0; j<4; ++j)
        {
            boxFace->push_back();
        }
        boxGeom->addPrimitiveSet(boxFace);
    }


    osg::ref_ptr<osg::Geode> box_node = new osg::Geode;

    
    addChild(box_node);
    
    return box_node;
}
