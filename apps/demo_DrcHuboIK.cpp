
#include "akin/Robot.h"
#include "HuboKin/DrcHubo.h"
#include "akin/Solver.h"

#include "akinUtils/urdfParsing.h"

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
//        drchubo = new Robot;
//        akinUtils::loadURDF(*drchubo, "../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
//                            "../../../resources/drchubo");
        drchubo = new DrcHubo(
                    "../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
                    "../../../resources/drchubo");
        
//        com_joints = Robot::Explorer::getJointIds(drchubo->joint(DOF_POS_X));
        com_joints = Robot::Explorer::getIdPath(drchubo->joint(DOF_POS_X), drchubo->joint(DOF_ROT_Z));
        
        comx = new CenterOfMassContraintX(com_joints.size(), *drchubo, com_joints);
        drchubo->manip(DrcHubo::MANIP_L_FOOT).mode = Manipulator::SUPPORT;

        addRobot(*drchubo);
        addRootFrame(Frame::World());
        
//        drchubo->joint(DOF_POS_X).value(1);

        drchubo->joint("LEP").value(-90*DEG);
        drchubo->joint("LWP").value(-90*DEG);

        drchubo->manip(DrcHubo::MANIP_L_HAND).mode = Manipulator::LINKAGE;
        lh_baseTf = drchubo->manip(DrcHubo::MANIP_L_HAND).respectToWorld();
        lh_config = drchubo->getConfig(drchubo->manip(DrcHubo::MANIP_L_HAND)
                                       .constraint().getJoints());

        drchubo->manip(DrcHubo::MANIP_R_FOOT).mode = Manipulator::ANALYTICAL;
        rf_baseTf = drchubo->manip(DrcHubo::MANIP_R_FOOT).respectToWorld();
        rf_joints = drchubo->manip(DrcHubo::MANIP_R_FOOT).constraint().getJoints();
        rf_config = drchubo->getConfig(rf_joints);
        
        std::vector<Eigen::Vector2d> poly = drchubo->getSupportPolygon();
        std::cout << "Support:\n";
        for(size_t i=0; i<poly.size(); ++i)
            std::cout << poly[i].transpose() << std::endl;
        
        std::cout << "Center: " << drchubo->getSupportCenter().transpose() << std::endl;

        time = 0;
    }

    virtual void update()
    {
        time += 0.01;

//        Transform lh_targetTf = lh_baseTf;
//        lh_targetTf.pretranslate( 0.1*Vec3(1,1,1) * (1-cos(time))/2);
//        lh_targetTf.rotate(Rotation( 90*DEG * (1-cos(time))/2, Vec3(1,0,0) ));
//        drchubo->manip(DrcHubo::MANIP_L_HAND).ik(lh_config, lh_targetTf, Frame::World());

        Transform rf_targetTf = rf_baseTf;
        rf_targetTf.pretranslate( 0.1*Vec3(2,-4,3) * (1-cos(time))/2);
        rf_targetTf.rotate(Rotation( -90*DEG * (1-cos(time))/2, Vec3(0,0,1)));
        rf_targetTf.rotate(Rotation( -45*DEG * (1-cos(time))/2, Vec3(0,1,0)));
        if(drchubo->manip(DrcHubo::MANIP_R_FOOT).ik(rf_config, rf_targetTf, Frame::World()))
            drchubo->setConfig(rf_joints, rf_config);
        
        com_config = drchubo->getConfig(com_joints);
        std::cout << comx->getJacobian(com_config) << "\n" << std::endl;

        AkinNode::update();
    }

    ~CustomNode()
    {
        delete drchubo;
    }

protected:

    double time;
    DrcHubo* drchubo;

    Transform lh_baseTf;
    Eigen::VectorXd lh_config;

    std::vector<size_t> rf_joints;
    Transform rf_baseTf;
    Eigen::VectorXd rf_config;
    
    std::vector<size_t> com_joints;
    Eigen::VectorXd com_config;
    
    CenterOfMassContraintX* comx;

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
