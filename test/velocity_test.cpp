
#include "../osgAkin/AkinCallback.h"
#include "osgGA/GUIEventHandler"
#include "../osgAkin/Line.h"
#include <osgGA/TrackballManipulator>

using namespace akin;
using namespace osgAkin;
using namespace std;

class CustomNode : public AkinNode
{
public:

    CustomNode() : elapsed_time(0), dt(0.001), report_time(0.01), iterations(0), paused(false)
    {
        geode = new osg::Geode;
        geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        addChild(geode);
    }

    virtual void update()
    {
        if(paused)
        {
            AkinNode::update();
            return;
        }

        elapsed_time += dt;

        if(elapsed_time >= report_time)
        {
            ++iterations;
            elapsed_time = 0;
//            Frame* frame = &getFrame(0);
//            cout << "(Iteration " << iterations << ")\n";
//            while(frame)
//            {
//                cout << "Frame " << frame->name() << ": <"
//                     << frame->linearVelocity().transpose() << "> | <"
//                     << frame->relativeLinearVelocity().transpose() << ">"
//                     << "\n";

//                if(frame->numChildFrames()==0)
//                    frame = NULL;
//                else
//                    frame = &frame->childFrame(0);
//            }

//            cout << "\n_________________" << endl;
//            cout << getFrame(0).childFrame(0).childFrame(0).linearVelocity().transpose()
//                 << "\t|\t"
//                 << getFrame(0).childFrame(0).linearVelocity().transpose()
//                 << endl;
        }

        Frame* frame = &getFrame(0);

        size_t counter = 0;
        while(frame)
        {
            const Transform& tf0 = frame->respectToRef();
            Velocity v = frame->relativeLinearVelocity();
            Velocity w = frame->relativeAngularVelocity();

            Transform tf;
            tf.translate(tf0.translation());
            tf.translate(v*dt);
            tf.rotate(Rotation(w.norm()*dt, w));
            tf.rotate(tf0.rotation());

            frame->respectToRef(tf);

            if(vels.size() == counter)
            {
                vels.push_back(new osgAkin::Line);
                vels[counter]->addVertex(Translation());
                vels[counter]->addVertex(Translation());
                vels[counter]->setColor(osg::Vec4(1,0,1,1));
                geode->addDrawable(vels[counter]);
            }

            vels[counter]->moveVertex(0, frame->respectToWorld().translation());
            vels[counter]->moveVertex(1, frame->respectToWorld().translation()
                                      + frame->linearVelocity()*dt*10);
            vels[counter]->updateVertices();

            Frame& follower = getFrame(counter+1);
            const Transform& tff = follower.respectToRef();
            tf.setIdentity();
            tf.translate(tff.translation());
            tf.translate(frame->linearVelocity()*dt);
            tf.rotate(Rotation(Velocity(frame->angularVelocity()*dt)));
            tf.rotate(tff.rotation());
            
            follower.respectToRef(tf);
            

            if(frame->numChildFrames()==0)
                frame = NULL;
            else
                frame = &frame->childFrame(0);

            ++counter;
        }

        AkinNode::update();
    }

    double elapsed_time;
    double dt;
    double report_time;
    size_t iterations;
    bool paused;
    std::vector<osgAkin::Line*> vels;
    osg::ref_ptr<osg::Geode> geode;

};

class CustomEventHandler : public osgGA::GUIEventHandler
{
public:

    CustomNode* myNode;

    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
            case osgGA::GUIEventAdapter::KEYDOWN:
                switch(ea.getKey())
                {
                    case ' ':
                        myNode->paused = !myNode->paused;
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

int main(int, char* [])
{
    osg::ref_ptr<CustomNode> node = new CustomNode;
    osg::ref_ptr<CustomEventHandler> hevent = new CustomEventHandler;
    hevent->myNode = node;

    Frame A(Frame::World(), "A");
    Frame B(Transform(Translation(0,0,0)), A, "B");
    Frame C(Transform(Translation(0,0.5,0)), B, "C");
    Frame D(Transform(Translation(1,0,0)), C, "D");

    node->addRootFrame(A);
    A.relativeAngularVelocity(Velocity(0,0,1));
    B.relativeLinearVelocity(Velocity(0.1,0,0));
    B.relativeAngularVelocity(Velocity(0,0,1));
    C.relativeAngularVelocity(Velocity(0,1,0));
    
    Frame Af(A.respectToWorld(), Frame::World(), "A follower");
    node->addRootFrame(Af);
    Frame Bf(B.respectToWorld(), Frame::World(), "B follower");
    node->addRootFrame(Bf);
    Frame Cf(C.respectToWorld(), Frame::World(), "C follower");
    node->addRootFrame(Cf);
    Frame Df(D.respectToWorld(), Frame::World(), "D follower");
    node->addRootFrame(Df);
    
    node->addRootFrame(Frame::World());

    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor(osg::Vec4(0.3,0.3,0.3,1.0));
    viewer.setSceneData(node);
    viewer.addEventHandler(hevent);

    viewer.setUpViewInWindow(0, 0, 640, 480);
    viewer.realize();

    osg::ref_ptr<osgGA::TrackballManipulator> cm = new osgGA::TrackballManipulator;
    cm->setHomePosition(osg::Vec3(0,0,5), osg::Vec3(0,0,0), osg::Vec3(0,1,0));
    viewer.setCameraManipulator(cm);

    viewer.run();

    return 0;
}
