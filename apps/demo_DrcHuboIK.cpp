
#include "akin/Robot.h"
#include "HuboKin/DrcHubo.h"
#include "akin/Solver.h"

#include "urdfAkin/urdfParsing.h"

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

        // Add drchubo into the scene so it is visible
        addRobot(*drchubo);
        
        // Set the left foot to support mode, so it will stand on just that foot
        drchubo->manip(DrcHubo::LEFT_FOOT).mode = Manipulator::SUPPORT;

        // Set the right foot into analytical solver mode and grab its initial transformation
        drchubo->manip(DrcHubo::RIGHT_FOOT).mode = Manipulator::ANALYTICAL;
        rf_baseTf = drchubo->manip(DrcHubo::RIGHT_FOOT).respectToWorld();
        
        // Make Hubo squat by 20cm
        drchubo->dof(DOF_POS_Z).position( drchubo->dof(DOF_POS_Z).position()-0.2 );

        // Move the left arm to a starting configuration
        drchubo->dof("LEP").position(-90*DEG);
        drchubo->dof("LWP").position(-90*DEG);

        // Move the right arm to a starting configuration
        drchubo->dof("RSR").position(-90*DEG);
        drchubo->dof("REP").position(-90*DEG);

        // Set the Left Hand to LINKAGE (7-DOF iterative) IK mode and grab its initial transform
        drchubo->manip(DrcHubo::LEFT_HAND).mode = Manipulator::LINKAGE;
        lh_baseTf = drchubo->manip(DrcHubo::LEFT_HAND).respectToWorld();
        
        // Print out the support polygon (no real reason for this)
        std::vector<Eigen::Vector2d> poly = drchubo->getSupportPolygon();
        std::cout << "Support:\n";
        for(size_t i=0; i<poly.size(); ++i)
            std::cout << poly[i].transpose() << std::endl;
        
        // Print out where the center of mass is (no real reason for this)
        std::cout << "Center: " << drchubo->getSupportCenter().transpose() << std::endl;

        // Change some solver settings
        drchubo->solver().max_steps = 50;
        drchubo->solver().max_attempts = 1;

        // Change the maximum x-translation limit for the Left Hand's Task Space Region
        drchubo->manip(DrcHubo::LEFT_HAND).constraint()->max_limits.linear()[0] = INFINITY;
        // Note: max_limits.linear()[0] can be replaced with max_limits[0]

        // Change the minimum x-rotation limit for the Left Hand's Task Space Region
        drchubo->manip(DrcHubo::LEFT_HAND).constraint()->min_limits.angular()[0] = -INFINITY;
        // Note: min_limits.angular()[0] can be replaced with min_limits[3]

        // Initialize time
        time = 0;
    }

    virtual void customUpdate()
    {
        // Update time
        time += 0.01;

        // Move the Left Hand Target
        Transform lh_targetTf = lh_baseTf;
        lh_targetTf.pretranslate( 0.1*Vec3(1,1,1) * (1-cos(time))/2);
        lh_targetTf.rotate(Rotation( 90*DEG * (1-cos(time))/2, Vec3(1,0,0) ));
        drchubo->manip(DrcHubo::LEFT_HAND).constraint()->target = lh_targetTf;

        // Move the Right Hand Target
        Transform rf_targetTf = rf_baseTf;
        rf_targetTf.pretranslate( 0.1*Vec3(2,-4,3) * (1-cos(time))/2);
        rf_targetTf.rotate(Rotation( -90*DEG * (1-cos(time))/2, Vec3(0,0,1)));
        rf_targetTf.rotate(Rotation( -45*DEG * (1-cos(time))/2, Vec3(0,1,0)));
        drchubo->manip(DrcHubo::RIGHT_FOOT).constraint()->target = rf_targetTf;

        // Simultaneously solve all manipulator constraints (including the balance constraint)
        drchubo->solve();
    }

    virtual ~CustomNode()
    {
        delete drchubo;
    }

protected:

    double time;
    DrcHubo* drchubo;

    Transform lh_baseTf;

    Transform rf_baseTf;

};

int main()
{
    osg::ref_ptr<CustomNode> node = new CustomNode;

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.3,0.3,0.3,1));
    viewer.setSceneData(node);

    viewer.getCamera()->getOrCreateStateSet()->setGlobalDefaults();

    viewer.run();
}
