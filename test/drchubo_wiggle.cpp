

#include "osgAkin/AkinNode.h"
#include "HuboKin/DrcHubo.h"
#include <osgDB/WriteFile>
#include <iomanip>
#include "akin/RobotConstraint.h"
#include "osgAkin/Line.h"
#include "osgAkin/AkinVisual.h"
#include "osg/Point"
#include "akin/Solver.h"

using namespace std;
using namespace akin;
using namespace osgAkin;
using namespace HuboKin;

class CustomNode : public AkinNode
{
public:

    bool paused;

    CustomNode() : paused(true), time(0)
    {
        drchubo = new DrcHubo(
                    "../../../resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf",
                    "../../../resources/drchubo");
        addRobot(*drchubo);

        drchubo->manip(DrcHubo::LEFT_FOOT).mode = Manipulator::SUPPORT;
//        drchubo->manip(DrcHubo::RIGHT_FOOT).mode = Manipulator::ANALYTICAL;
        drchubo->manip(DrcHubo::RIGHT_FOOT).mode = Manipulator::SUPPORT;

        drchubo->manip(DrcHubo::LEFT_HAND).mode = Manipulator::LINKAGE;
        drchubo->manip(DrcHubo::RIGHT_HAND).mode = Manipulator::LINKAGE;

        drchubo->dof(DOF_POS_Z).position( drchubo->dof(DOF_POS_Z).position()-0.2 );
        drchubo->dof("LSP").position(  20*DEG);
        drchubo->dof("LSR").position(  20*DEG);
        drchubo->dof("LWR").position(  20*DEG);
        drchubo->dof("LEP").position(-110*DEG);
        drchubo->dof("RSP").position(  20*DEG);
        drchubo->dof("RSR").position( -20*DEG);
        drchubo->dof("REP").position(-110*DEG);
        drchubo->dof("RWR").position( -20*DEG);

        drchubo->manip(DrcHubo::LEFT_HAND).constraint()->target =
                drchubo->manip(DrcHubo::LEFT_HAND).respectToWorld();
        drchubo->manip(DrcHubo::RIGHT_HAND).constraint()->target =
                drchubo->manip(DrcHubo::RIGHT_HAND).respectToWorld();

        z_base = drchubo->dof(DOF_POS_Z).position();

        drchubo->solver().max_steps = 50;

        for(size_t i=0; i<drchubo->numManips(); ++i)
            drchubo->manip(i).addVisual(Geometry());
    }

    virtual void customUpdate()
    {
        if(paused)
            return;

        time += 0.05;

        drchubo->dof(DOF_POS_Y).position( 0.05*sin(time) );
        drchubo->dof(DOF_POS_Z).position( 0.05*cos(time) + z_base );

        drchubo->solve();
    }


protected:

    double time;
    double r;
    double z_base;
    DrcHubo* drchubo;

};

class SaveScreen : public osg::Camera::DrawCallback
{
public:

    SaveScreen() : paused(true), count(0) { }

    bool paused;
    std::string save_dir;
    osg::ref_ptr<osg::Camera> vcamera;

    virtual void operator () (osg::RenderInfo& renderInfo) const
    {
        osg::Camera::DrawCallback::operator ()(renderInfo);

        if(save_dir.empty())
            return;

        if(paused)
            return;

        ++count;
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

protected:
    mutable size_t count;

};

class CustomEventHandler : public osgGA::GUIEventHandler
{
public:

    osg::ref_ptr<SaveScreen> ss;
    osg::ref_ptr<CustomNode> cn;

    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
            case osgGA::GUIEventAdapter::KEYDOWN:
                switch(ea.getKey())
                {
                    case ' ':
                        ss->paused = !ss->paused;
                        cn->paused = !cn->paused;
                        return true;
                    default:
                        return false;
                }
                break;
            default:
                return false;
        }
    }

    virtual void accept(osgGA::GUIEventHandlerVisitor& v) { v.visit(*this); }
};

int main(int argc, char* argv[])
{
    osg::ref_ptr<SaveScreen> ss = new SaveScreen;
    for(int i=1; i<argc; ++i)
        ss->save_dir = argv[i];

    osg::ref_ptr<CustomNode> node = new CustomNode;

    osg::ref_ptr<CustomEventHandler> ces = new CustomEventHandler;
    ces->ss = ss;
    ces->cn = node;

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.05,0.05,0.05,1));

    viewer.setSceneData(node);
    viewer.addEventHandler(ces);

    ss->vcamera = viewer.getCamera();
    viewer.getCamera()->setFinalDrawCallback(ss);

    viewer.setUpViewInWindow(0,0,640,480);

    viewer.run();
}
