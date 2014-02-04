
#include "AkinVisual.h"
#include "osgDB/ReadFile"
#include "osg/ShapeDrawable"

using namespace akin;
using namespace osgAkin;


AkinVisual::AkinVisual(const Geometry &visual)
{
    _initializeVisual(visual);
}


void AkinVisual::_initializeVisual(const Geometry &visual)
{
    setMatrix(cosg(visual.relative_pose));

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
    else if(visual.type == Geometry::SPHERE)
    {
        addChild(_makeSphere(visual));

//        osg::Sphere* sphere = new osg::Sphere(osg::Vec3(visual.relative_pose.translation()[0],
//                                                        visual.relative_pose.translation()[1],
//                                                        visual.relative_pose.translation()[2]),
//                                                visual.scale[0]);
//        osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere);
//        sphereDrawable->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));

//        osg::ref_ptr<osg::Geode> sphere_node = new osg::Geode;
//        addChild(sphere_node);
//        sphere_node->addDrawable(sphereDrawable);
//        std::cout << "We have a sphere!" << std::endl;
    }
    else if(visual.type == Geometry::BOX)
    {
        addChild(_makeBox(visual));

//        osg::Box* box = new osg::Box(osg::Vec3(visual.relative_pose.translation()[0],
//                                               visual.relative_pose.translation()[1],
//                                               visual.relative_pose.translation()[2]),
//                visual.scale[0], visual.scale[1], visual.scale[2]);
//        osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box);
//        boxDrawable->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));

//        osg::ref_ptr<osg::Geode> box_node = new osg::Geode;
//        addChild(box_node);
//        box_node->addDrawable(boxDrawable);
//        std::cout << "We have a box!" << std::endl;
    }
}


osg::Geode* AkinVisual::_makeSphere(const Geometry &visual)
{

}

