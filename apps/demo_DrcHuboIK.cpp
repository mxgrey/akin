
#include "akin/Robot.h"
#include "HuboKin/DrcHubo.h"
#include "akin/Solver.h"

#include "akinUtils/urdfParsing.h"

#include "osgAkin/AkinCallback.h"

using namespace akin;
using namespace osgAkin;

class CustomNode : public AkinNode
{
public:

    CustomNode()
    {
//        drchubo = new Robot;
//        akinUtils::loadURDF(*drchubo, "../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
//                            "../../../resources/drchubo");
        drchubo = new HuboKin::DrcHubo(
                    "../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
                    "../../../resources/drchubo");

        addRobot(*drchubo);
        addRootFrame(Frame::World());

//        drchubo->joint(DOF_POS_Z).value(
//                    -drchubo->link("leftFoot").respectToWorld().translation()[2]);

//        m = drchubo->addManipulator(drchubo->joint("LWR").childLink(), "leftHandManip",
//                                        drchubo->link("leftPalm").respectToRef());

        drchubo->joint("LEP").value(-90*DEG);
        drchubo->joint("LWP").value(-90*DEG);
        baseTf = drchubo->manip(m).respectToWorld();
        config = drchubo->getConfig(drchubo->manip(m).constraint(Manipulator::LINKAGE).getJoints());

        time = 0;
    }

    virtual void update()
    {
        time += 0.01;

        Transform targetTf = baseTf;
        targetTf.pretranslate( 0.1*Vec3(1,1,1) * (1-cos(time))/2);
        targetTf.rotate(Rotation( 90*DEG * (1-cos(time))/2, Vec3(1,0,0) ));
        drchubo->manip(m).ik(config, targetTf);

        AkinNode::update();
    }

    ~CustomNode()
    {
        delete drchubo;
    }

protected:

    double time;
    HuboKin::DrcHubo* drchubo;
    int m;
    Manipulator::Mode mode;
    Transform baseTf;
    Eigen::VectorXd config;
};

int main(int , char* [])
{
    osg::ref_ptr<osg::Group> root = new osg::Group;
    osg::ref_ptr<CustomNode> node = new CustomNode;
    root->addChild(node);

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.3,0.3,0.3,1));
    viewer.setSceneData(root);

    viewer.run();

    return 0;
}
