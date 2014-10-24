
#include "akin/Robot.h"
#include "HuboKin/DrcHubo.h"
#include "akin/Solver.h"

#include "osgAkin/AkinCallback.h"

using namespace std;
using namespace akin;
using namespace HuboKin;
using namespace osgAkin;


class CustomNode : public AkinNode
{
public:

    CustomNode()
    {
        drchubo = new DrcHubo(
                    "../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
                    "../../../resources/drchubo");

        drchubo->setDynamicsMode(FORWARD);

        time = 0;
        dt = 0.01;
    }

    void simulate_step()
    {
//        ex.reset(drchubo->dof(DOF_POS_X));
//        Joint* joint;

//        while( (joint = ex.nonconst_nextJoint()) )
//            joint->velocity( joint->acceleration()*dt );
    }

    virtual void customUpdate()
    {
        simulate_step();
    }



protected:

    double time;
    double dt;
    DrcHubo* drchubo;
    Robot::Explorer ex;

};

int main()
{
    Frame::World().gravity(Eigen::Vector3d::Zero(), Frame::World());

    osg::ref_ptr<CustomNode> node = new CustomNode;

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.3,0.3,0.3,1));
    viewer.setSceneData(node);

    viewer.getCamera()->getOrCreateStateSet()->setGlobalDefaults();

    viewer.run();
}
