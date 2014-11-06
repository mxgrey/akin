
#include "osgAkin/AkinNode.h"
#include "akin/Robot.h"
#include <osgDB/WriteFile>
#include <iomanip>


using namespace std;
using namespace akin;
using namespace osgAkin;



void createSimpleRobot(Robot& robot)
{
    Geometry c;
    c.type = Geometry::CYLINDER;
    c.scale = Eigen::Vector3d(0.05,0.05,0.1);
    c.colors.push_back(ColorSpec::White());
    c.relative_pose.rotate(Rotation(90*DEG,Vec3(1,0,0)));
    Geometry b1;
    b1.type = Geometry::BOX;
    double L1;
    b1.scale = Eigen::Vector3d(0.05,0.05,L1);
    b1.colors.push_back(ColorSpec::White());
    b1.relative_pose.translate(Translation(0,0,L1/2));

    robot.link(0).addVisual(c);
    robot.link(0).addVisual(b1);

    Geometry b2 = b1;
    double L2 = L1/2;
    b2.scale[2] = L2;
    b2.relative_pose.translation()[2] = L2/2;
    ProtectedJointProperties p;
    p._axis = Vec3(0,1,0);
    p._name = "joint1";
    p._type = Joint::REVOLUTE;
    p._baseTransform.translate(Vec3(0,0,L1));
    robot.createJointLinkPair(0, "link2", p, DofProperties());
    robot.link(1).addVisual(c);
    robot.link(1).addVisual(b2);

    p._name = "joint2";
    p._baseTransform.translation()[2] = L2;
    robot.createJointLinkPair(1, "link3", p, DofProperties());
    robot.link(2).addVisual(c);
    robot.link(2).addVisual(b1);
}


int main()
{
    Robot robot;
    createSimpleRobot(robot);

    osg::ref_ptr<AkinNode> node = new AkinNode;
    node->addRobot(robot);

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.05,0.05,0.05,1));
    viewer.setSceneData(node);

    viewer.setUpViewInWindow(0,0,640,480);

    viewer.run();
}
