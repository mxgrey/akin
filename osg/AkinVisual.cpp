
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

//osg::Geode* AkinVisual::_makeBox(const Geometry &visual)
//{
    // TODO: Come up with a decent way of rendering a primitive box
    
//    osg::Box* box = new osg::Box(osg::Vec3(visual.relative_pose.translation()[0],
//                                           visual.relative_pose.translation()[1],
//                                           visual.relative_pose.translation()[2]),
//            visual.scale[0], visual.scale[1], visual.scale[2]);
//    osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box);
    
    
////    cout << "Pushing back color" << endl;
//    if(visual.colors.size() > 0)
//        _colors->push_back(cosg(visual.colors[0]));
//    else
//        _colors->push_back(cosg(akin::ColorSpec::Gray()));
        
////    boxDrawable->setUseVertexBufferObjects(true);
//    boxDrawable->setColor((*_colors)[0]);
    
//    osg::ref_ptr<osg::Geode> box_node = new osg::Geode;
//    box_node->addDrawable(boxDrawable);
    
//    addChild(box_node);
    
//    return box_node;
//}
