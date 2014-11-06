
#include "osgAkin/AkinCallback.h"
#include "HuboKin/DrcHubo.h"
#include <random>
#include <osgDB/WriteFile>
#include <iomanip>

using namespace std;
using namespace akin;
using namespace HuboKin;
using namespace osgAkin;

class SaveScreen : public osg::Camera::DrawCallback
{
public:

    SaveScreen() : count(0) { }

    size_t count;
    std::string save_dir;
    osg::ref_ptr<osg::Camera> vcamera;

    virtual void operator () (osg::RenderInfo& renderInfo) const
    {
        osg::Camera::DrawCallback::operator ()(renderInfo);

        if(save_dir.empty())
            return;

        int x,y;
        unsigned int width,height;
        osg::ref_ptr<osg::Viewport> vp = vcamera->getViewport();
        x = vp->x();
        y = vp->y();
        width = vp->width();
        height = vp->height();

        osg::ref_ptr<osg::Image> image = new osg::Image;
        image->readPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE);

        std::stringstream str;
        str << save_dir << "/image" << std::setfill('0') << setw(6) << count
            << setw(0) << ".png";

        if(osgDB::writeImageFile(*image,str.str()))
            std::cout << "Saving image to " << str.str() << std::endl;
        else
            std::cout << "Image saving failed!" << std::endl;
    }
};

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
        ++ss->count;

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

    osg::ref_ptr<SaveScreen> ss;

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

int main(int argc, char* argv[])
{
    osg::ref_ptr<SaveScreen> ss = new SaveScreen;
    for(int i=1; i<argc; ++i)
        ss->save_dir = argv[i];


    osg::ref_ptr<CustomNode> node = new CustomNode;
    node->ss = ss;

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.05,0.05,0.05,1));
    viewer.setSceneData(node);

    viewer.getCamera()->getOrCreateStateSet()->setGlobalDefaults();
    ss->vcamera = viewer.getCamera();
    viewer.getCamera()->setFinalDrawCallback(ss);

    viewer.setUpViewInWindow(0,0,640,480);

    viewer.run();
}
