#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/LineSegment>
#include <osgViewer/Viewer>
#include <osg/LineWidth>
#include <osg/MatrixTransform>

#include "AkinCallback.h"
#include "Axes.h"

#include "Link.h"

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
    Robot* rb_ptr = new Robot(Robot::URDF_FILE, "../models/drchubo_v2.urdf",
                              Frame::World(), verbosity::DEBUG);
    Robot& robot = *rb_ptr;
    
    return robot;
}

void display_robot(Robot& displaying_robot)
{
    osg::Group* root = new osg::Group;
    osgAkin::AkinNode* akinNode = new osgAkin::AkinNode;
    root->addChild(akinNode);
    
    akinNode->addRootFrame(displaying_robot.link(0));
    
    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.3f,0.3f,0.3f,1.0f));
    viewer.setSceneData(root);
    viewer.run();
}


int main(int argc, char* argv[])
{
//    Robot& built_robot = build_manual_robot();
    
    Robot& built_robot = build_urdf_robot();
    
    display_robot(built_robot);
}
