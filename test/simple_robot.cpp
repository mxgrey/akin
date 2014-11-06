
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
    Geometry b;
    b.type = Geometry::BOX;
    b.scale = Eigen::Vector3d(0.05,0.05,1);
    b.colors.push_back(ColorSpec::White());
    b.relative_pose.translate(Translation(0,0,b.scale[2]/2));

    robot.link(0).addVisual(c);
    robot.link(0).addVisual(b);


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
