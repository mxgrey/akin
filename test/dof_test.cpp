
#include "osgAkin/AkinCallback.h"
#include "HuboKin/DrcHubo.h"
#include <random>

using namespace std;
using namespace akin;
using namespace HuboKin;
using namespace osgAkin;

class CustomNode : public AkinNode
{
public:

    CustomNode() : time(0), waypoint(0)
    {
        drchubo = new DrcHubo(
                    "../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
                    "../../../resources/drchubo");

        addRobot(*drchubo);

        for(size_t i=0; i<drchubo->numManips(); ++i)
            drchubo->manip(i).addVisual(Geometry());

        velocity = 10;

        std::random_device rd;
        std::mt19937 generator(rd());

        for(size_t i=0; i<3; ++i)
        {
            drchubo->dof(i).min(-0.2);
            drchubo->dof(i).max( 0.2);
        }

        for(size_t i=3; i<6; ++i)
        {
            drchubo->dof(i).min(-M_PI/2);
            drchubo->dof(i).max( M_PI/2);
        }

        numWaypoints = 1000;
        trajectory.resize(numWaypoints, Eigen::VectorXd(drchubo->numDofs()));

        for(size_t j=0; j<drchubo->numDofs(); ++j)
        {
            std::uniform_real_distribution<double> distribution(drchubo->dof(j).min(),
                                                                drchubo->dof(j).max());
            for(size_t i=0; i<numWaypoints; ++i)
                trajectory[i][j] = distribution(generator);
        }

        step_forward();
    }

    void step_forward()
    {
        ++waypoint;
        period = 0;
        for(size_t j=0; j<drchubo->numDofs(); ++j)
            period = std::max(period, (trajectory[waypoint][j]-trajectory[waypoint-1][j])/velocity);
        time = 0;
    }

    virtual void customUpdate()
    {
        time += 0.01;

        if(time >= period)
            step_forward();

        const Eigen::VectorXd& w0 = trajectory[waypoint-1];
        const Eigen::VectorXd& w1 = trajectory[waypoint];

        config = (w1-w0)/period*time + w0;

        drchubo->setConfig(config);
    }

    virtual ~CustomNode()
    {
        delete drchubo;
    }


protected:

    DrcHubo* drchubo;

    double time;
    double period;
    double velocity;
    size_t waypoint;
    size_t numWaypoints;
    Eigen::VectorXd config;
    std::vector<Eigen::VectorXd> trajectory;

};

int main()
{
    osg::ref_ptr<CustomNode> node = new CustomNode;
//    Geometry G;
//    G.type = Geometry::BOX;
//    G.scale = Eigen::Vector3d(5,5,0.001);
//    G.colors.push_back(ColorSpec());
//    Frame F(Transform(Translation(0,0,-0.5), Rotation()));
//    F.addVisual(G);
//    node->addRootFrame(F);

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.05,0.05,0.05,1));
    viewer.setSceneData(node);

    viewer.getCamera()->getOrCreateStateSet()->setGlobalDefaults();

    viewer.setUpViewInWindow(0,0,640,480);

    viewer.run();
}
