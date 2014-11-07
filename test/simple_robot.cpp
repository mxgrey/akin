
#include "osgAkin/AkinNode.h"
#include "akin/Robot.h"
#include <osgDB/WriteFile>
#include <iomanip>
#include "akin/RobotConstraint.h"
#include "osgAkin/Line.h"
#include "osgAkin/AkinVisual.h"
#include "osg/Point"

using namespace std;
using namespace akin;
using namespace osgAkin;

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

void createSimpleRobot(Robot& robot)
{
    Geometry c;
    c.type = Geometry::CYLINDER;
    c.scale = Eigen::Vector3d(0.05,0.05,0.1);
    c.colors.push_back(ColorSpec::White());
    c.relative_pose.rotate(Rotation(90*DEG,Vec3(1,0,0)));
    Geometry b1;
    b1.type = Geometry::BOX;
    double L1 = 1;
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
    p._name = "r1";
    p._type = Joint::REVOLUTE;
    p._baseTransform.translate(Vec3(0,0,L1));
    robot.createJointLinkPair(0, "link2", p, DofProperties());
    robot.link(1).addVisual(c);
    robot.link(1).addVisual(b2);

    p._name = "r2";
    p._baseTransform.translation()[2] = L2;
    robot.createJointLinkPair(1, "link3", p, DofProperties());
    robot.link(2).addVisual(c);
    robot.link(2).addVisual(b1);

    c.relative_pose.translation()[2] = L1;
    robot.link(2).addVisual(c);

    robot.addManipulator(robot.link(2), "foot", Transform(Vec3(0,0,L1),Rotation()));
    robot.setDefaultTaskConstraint();
    robot.manip(0).mode = Manipulator::LINKAGE;
    robot.manip(0).addVisual(Geometry());
    for(size_t i=0; i<3; ++i)
    {
        robot.manip(0).constraint()->min_limits.angular()[i] = -INFINITY;
        robot.manip(0).constraint()->max_limits.angular()[i] =  INFINITY;
    }

    robot.dof("r1").position(90*DEG);
    robot.dof("r2").position(90*DEG);
}


class CustomNode : public AkinNode
{
public:

    bool paused;
    bool plot;

    CustomNode() : paused(true), plot(false), time(0)
    {
        createSimpleRobot(robot);
        addRobot(robot);
        config = robot.getConfig(robot.manip(0).constraint()->getDofs());
        min_vals = Vec3(robot.dof(DOF_ROT_Y).position(),config[0],config[1]);
        max_vals = min_vals;

        baseTf = robot.manip(0).respectToWorld();
        r = 15*DEG;
    }

    void append_to_path()
    {
        Vec3 newV(robot.dof(DOF_ROT_Y).position(),config[0],config[1]);
        path->addVertex(newV);
        path->updateVertices();

        for(size_t i=0; i<3; ++i)
        {
            min_vals[i] = std::min(min_vals[i], newV[i]);
            max_vals[i] = std::max(max_vals[i], newV[i]);
        }
    }

    void setup_plot()
    {
        plot = true;
        path = new osgAkin::Line;
        path->setColor(osg::Vec4(0,0,0,1));
        osg::ref_ptr<osg::Geode> plotGeode = new osg::Geode;
        plotGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::DYNAMIC);
        plotGeode->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(10.0f));
        plotGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        addChild(plotGeode);
        plotGeode->addDrawable(path);

        point = new osg::Geometry;
        plotGeode->addDrawable(point);

        p = new osg::Vec3Array;
        p->resize(1);
        point->setVertexArray(p);
        osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
        c->push_back(osg::Vec4(237.0/255.0,145.0/255.0,33.0/255.0,1.0));
        point->setColorArray(c, osg::Array::BIND_OVERALL);
        osg::ref_ptr<osg::DrawElementsUShort> ele =
                new osg::DrawElementsUShort(osg::PrimitiveSet::POINTS);
        ele->push_back(0);
        point->addPrimitiveSet(ele);

        time = 0;
        while(time < 2*M_PI)
        {
            time += 0.05;

            robot.dof(DOF_ROT_Y).position(r*sin(time));
            robot.manip(0).ik(config,baseTf);

            append_to_path();
        }

        Geometry ag;
        ag.type = Geometry::AXES;
        ag.scale = (max_vals-min_vals)*1.2;
        ag.relative_pose.translate(min_vals-0.1*(max_vals-min_vals));
        osg::ref_ptr<osgAkin::AkinVisual> av = new osgAkin::AkinVisual(ag);
        addChild(av);

        time = 0;
    }

    void render_plot()
    {
        (*p)[0] = osg::Vec3(robot.dof(DOF_ROT_Y).position(),config[0],config[1]);
        point->setVertexArray(p);
    }

    virtual void update()
    {
        if(!paused)
        {
            time += 0.05;

            robot.dof(DOF_ROT_Y).position(r*sin(time));
            robot.manip(0).ik(config, baseTf);
        }

        // TODO: Figure out why this was failing
        // Best guess: NULL balance constraint makes it fail
//        Transform tf = baseTf;
//        tf.pretranslate(Vec3(r*sin(time),0,0));
//        robot.manip(0).constraint()->target = tf;
//        if(robot.manip(0).ik(config, tf))
//            std::cout << "Solved" << std::endl;
//        else
//            std::cout << "Failed" << std::endl;

        if(plot)
            render_plot();
        else
            AkinNode::update();
    }

protected:

    Vec3 min_vals;
    Vec3 max_vals;
    osg::ref_ptr<osgAkin::Line> path;
    osg::ref_ptr<osg::Geometry> point;
    osg::ref_ptr<osg::Vec3Array> p;
    Eigen::VectorXd config;
    double time;
    Robot robot;
    Transform baseTf;
    double r, T;

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
    bool plot = false;
    for(int i=1; i<argc; ++i)
    {
        if(string(argv[i])=="-p")
            plot = true;
        else
            ss->save_dir = argv[i];
    }

    osg::ref_ptr<CustomNode> node = new CustomNode;
    if(plot)
        node->setup_plot();

    osg::ref_ptr<CustomEventHandler> ces = new CustomEventHandler;
    ces->ss = ss;
    ces->cn = node;


    osgViewer::Viewer viewer;
    if(plot)
        viewer.getCamera()->setClearColor(osg::Vec4(1,1,1,1));
    else
        viewer.getCamera()->setClearColor(osg::Vec4(0.05,0.05,0.05,1));
    viewer.setSceneData(node);
    viewer.addEventHandler(ces);

    ss->vcamera = viewer.getCamera();
    viewer.getCamera()->setFinalDrawCallback(ss);

    viewer.setUpViewInWindow(0,0,640,480);

    viewer.run();
}
