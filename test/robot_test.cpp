#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/LineSegment>
#include <osgViewer/Viewer>
#include <osg/LineWidth>
#include <osg/MatrixTransform>

#include "osgAkin/AkinCallback.h"
#include "osgAkin/Axes.h"

#include "akin/Link.h"

using namespace akin;
using namespace std;

Robot& build_manual_robot()
{
    Robot* rb_ptr = new Robot(Robot::MANUAL);
    Robot& robot = *rb_ptr;
    
    robot.name("test_bot");
    
//    if(!robot.createRootLink("root_link"))
//    {
//        cout << "Wtf failed to create root link" << endl;
//        return robot;
//    }
    cout << "Size of links: " << robot.numLinks() << endl;
    
    int newID = robot.createJointLinkPair(0, "first_link", "first_joint", Transform(Translation(1,0,0)),
                                          Axis(1,0,0), Joint::REVOLUTE, -M_PI, M_PI);
    
    cout << "created first pair" << endl;
    
    cout << robot.link(0) << endl;
    cout << robot.link(1) << endl;
    
    if(newID <= 0)
    {
        cout << "Wtf invalid creation attempt" << endl;
        return robot;
    }
    
    newID = robot.createJointLinkPair(newID, "second_link", "second_joint", Transform(Translation(0,0,1)),
                                      Axis(0,0,1), Joint::PRISMATIC, -10, 10);
    
    robot.joint(0).value(180*M_PI/180);
    robot.joint(1).value(2);
    
    cout << robot.link(2) << endl;
    
    return robot;
}

Robot& build_urdf_robot()
{
    StringArray con_info;
    con_info.push_back("../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf");
    con_info.push_back("../../../resources/drchubo");

    Robot* rb_ptr = new Robot(Robot::URDF_FILE, con_info,
                              Frame::World(), verbosity::DEBUG);
    Robot& robot = *rb_ptr;

//    for(size_t i=0; i<robot.numLinks(); ++i)
//        if(robot.link(i).peekVisual(0).type == Geometry::MESH_FILE)
//            std::cout << "Mesh file for Link '" << robot.link(i).name()
//                  << "': " << robot.link(i).peekVisual(0).mesh_filename << std::endl;
    
    return robot;
}

void display_robot(Robot& displaying_robot)
{
    osg::Group* root = new osg::Group;
    osgAkin::KneeNode* akinNode = new osgAkin::KneeNode;
    root->addChild(akinNode);

    akinNode->addRobot(displaying_robot);

//    KinObject sphere(displaying_robot.joint("RWR").childLink(),"sphere",verbosity::INHERIT,
//                     "sphere",false);
//    akin::Geometry sGeom;
//    sGeom.type = Geometry::SPHERE;
//    sGeom.scale[0] = 0.1;
//    sGeom.colors.push_back(ColorSpec::Blue());
//    sphere.addVisual(sGeom);

//    KinObject box(displaying_robot.joint("LWR").childLink(),"box",verbosity::INHERIT,
//                  "box",false);
//    akin::Geometry bGeom;
//    bGeom.type = Geometry::BOX;
//    bGeom.scale = Eigen::Vector3d::Ones()*0.1;
//    bGeom.colors.push_back(ColorSpec::Red());
//    box.addVisual(bGeom);
    
    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.3f,0.3f,0.3f,1.0f));
    viewer.setSceneData(root);
    viewer.run();
}


int main(int argc, char* argv[])
{
//    Robot& built_robot = build_manual_robot();
    
    Robot& built_robot = build_urdf_robot();

    Frame* camera_frame = new Frame(akin::Transform(Translation(0,0,0),
                                                    Rotation(90*M_PI/180, akin::Axis(0, 1, 0))),
                                                    built_robot.joint("NK2").childLink(),
                                    "camera_frame");

    
    display_robot(built_robot);
}
