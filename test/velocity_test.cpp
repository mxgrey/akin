
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

            // Print messages here if desired
        }
        
        const Transform& tfr = referenceFrame->respectToRef();
        Velocity v = referenceFrame->relativeLinearVelocity();
        Velocity w = referenceFrame->relativeAngularVelocity();
        Transform tf;
        tf.translate(tfr.translation());
        tf.translate(v*dt);
        tf.rotate(Rotation(Velocity(w*dt)));
        tf.rotate(tfr.rotation());
        referenceFrame->respectToRef(tf);

        Frame* frame = &getFrame(0);

        size_t counter = 0;
        while(frame)
        {
            const Transform& tf0 = frame->respectToRef();
            v = frame->relativeLinearVelocity();
            w = frame->relativeAngularVelocity();

            tf.setIdentity();
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

            Frame& follower = *followers[counter];
            const Transform& tff = follower.respectToRef();
            tf.setIdentity();
            tf.translate(tff.translation());
            tf.translate(frame->linearVelocity(*referenceFrame)*dt);
            tf.rotate(Rotation(Velocity(frame->angularVelocity(*referenceFrame)*dt)));
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
    
    Frame* referenceFrame;
    std::vector<Frame*> followers;

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
    
//    node->referenceFrame = &Frame::World();
    Frame ref(Frame::World(), "Ref");
    node->referenceFrame = &ref;
    ref.respectToRef(Transform(Translation(0,1,1),Rotation(20*DEG,Axis(1,1,0))));
    ref.relativeLinearVelocity(Velocity(0,0,-0.2));
    ref.relativeAngularVelocity(Velocity(1,0.2,-0.3));

    Frame A(Frame::World(), "A");
    Frame B(Transform(Translation(0,0,0)), A, "B");
    Frame C(Transform(Translation(0,0.5,0)), B, "C");
    Frame D(Transform(Translation(1,0,0)), C, "D");

    node->addRootFrame(A);
    A.relativeAngularVelocity(Velocity(0,0,1));
    B.relativeLinearVelocity(Velocity(0.2,0.2,0.05));
    B.relativeAngularVelocity(Velocity(1,1,1));
    C.relativeAngularVelocity(Velocity(0,1,0));
    
    Frame Af(A.withRespectTo(*node->referenceFrame), *node->referenceFrame, "A follower");
    node->followers.push_back(&Af);
    Frame Bf(B.withRespectTo(*node->referenceFrame), *node->referenceFrame, "B follower");
    node->followers.push_back(&Bf);
    Frame Cf(C.withRespectTo(*node->referenceFrame), *node->referenceFrame, "C follower");
    node->followers.push_back(&Cf);
    Frame Df(D.withRespectTo(*node->referenceFrame), *node->referenceFrame, "D follower");
    node->followers.push_back(&Df);
    
    node->addRootFrame(*node->referenceFrame);

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
