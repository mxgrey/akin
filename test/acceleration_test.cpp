

#include "../osgAkin/AkinCallback.h"
#include "osgGA/GUIEventHandler"
#include "../osgAkin/Line.h"
#include <osgGA/TrackballManipulator>

using namespace akin;
using namespace osgAkin;
using namespace std;

typedef Eigen::Matrix<double,12,1> FrameDerivative;

class FrameState
{
public:

    Transform x;
    Screw v;

    FrameState& integrate(const FrameDerivative& f_dt)
    {
        Transform xi;
        xi.translate(x.translation());
        xi.translate(f_dt.block<3,1>(0,0));
        xi.rotate(Rotation(FreeVector(f_dt.block<3,1>(3,0))));
        xi.rotate(x.rotation());

        x = xi;

        v += f_dt.block<6,1>(6,0);

        return *this;
    }
};

typedef std::vector<FrameState> StateVector;
typedef std::vector<FrameDerivative> DerivVector;

DerivVector operator+(const DerivVector& dv1, const DerivVector& dv2)
{
    assert( dv1.size() == dv2.size() );
    DerivVector result(dv1.size());

    for(size_t i=0; i<dv1.size(); ++i)
        result[i] = dv1[i]+dv2[i];

    return result;
}

DerivVector operator*(const DerivVector& dv, double dt)
{
    DerivVector result(dv.size());
    for(size_t i=0; i<dv.size(); ++i)
        result[i] = dv[i]*dt;
    
    return result;
}

StateVector integrate(const StateVector& sv, const DerivVector& f_dt)
{
    assert( sv.size() == f_dt.size() );

    StateVector result = sv;

    for(size_t i=0; i<result.size(); ++i)
    {
        result[i].integrate(f_dt[i]);
    }

    return result;
}

void setStates(const std::vector<Frame*>& targets, const StateVector& sv)
{
    assert( targets.size() == sv.size() );
    for(size_t i=0; i<sv.size(); ++i)
    {
        Frame& F = *targets[i];
        F.respectToRef(sv[i].x);
        F.relativeVelocity(sv[i].v);
    }
}

void getStates(const std::vector<Frame*>& targets, StateVector& sv)
{
    assert( targets.size() == sv.size() );
    for(size_t i=0; i<sv.size(); ++i)
    {
        Frame& F = *targets[i];
        sv[i].x = F.respectToRef();
        sv[i].v = F.relativeVelocity();
    }
}

DerivVector getRelativeDerivatives(const std::vector<Frame*>& targets,
                                   const std::vector<Frame*>& followers,
                                   const Frame& reference,
                                   const StateVector& sv,
                                   const StateVector& svf,
                                   bool useVelocity=false)
{
    DerivVector dv(sv.size());

    setStates(targets, sv);
    setStates(followers, svf);

    if(useVelocity)
    {
        for(size_t i=0; i<dv.size(); ++i)
        {
            Frame& T = *targets[i];
            dv[i].block<6,1>(0,0) = T.velocity(reference);
            dv[i].block<6,1>(6,0) = T.acceleration(reference);
        }
    }
    else
    {
        for(size_t i=0; i<dv.size(); ++i)
        {
            Frame& T = *targets[i];
            Frame& F = *followers[i];
            dv[i].block<6,1>(0,0) = F.velocity(reference);
            dv[i].block<6,1>(6,0) = T.acceleration(reference);
        }
    }

    return dv;
}

DerivVector getPersonalDerivatives(const std::vector<Frame*> targets,
                                   const StateVector& sv)
{
    DerivVector dv(sv.size());

    setStates(targets, sv);

    for(size_t i=0; i<dv.size(); ++i)
    {
        Frame& F = *targets[i];
        dv[i].block<6,1>(0,0) = F.relativeVelocity();
        dv[i].block<6,1>(6,0) = F.relativeAcceleration();
    }

    return dv;
}

class CustomNode : public AkinNode
{
public:

    CustomNode() : elapsed_time(0), dt(0.001), report_time(0.1), iterations(0), paused(false)
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
//            elapsed_time = -report_time;
            
            std::cout << "Timestamp #" << iterations << "\n";
            
            for(size_t i=0; i<targets.size(); ++i)
            {
                Frame& T = *targets[i];
                T.relativeAcceleration(-T.relativeAcceleration());
                
                Frame& F = *followers[i];
                Screw diff = T.respectToWorld().diff(F.respectToWorld());
                std::cout << "(" << diff.norm() << ")\t" << diff.transpose() << "\n";
            }
            std::cout << std::endl;

            // Print messages here if desired
            
        }

//        std::cout << targets[1]->name() << " start\t"
//                  << targets[1]->acceleration(*referenceFrame).transpose()
//                  << std::endl;
        
        sv.resize(targets.size());
        getStates(targets, sv);

        svf.resize(followers.size());
        getStates(followers, svf);

//        std::cout << "RK1" << std::endl;
        k[0] = getPersonalDerivatives(targets, sv);
        kf[0] = getRelativeDerivatives(targets, followers, *referenceFrame, sv, svf);
        
//        std::cout << "RK2" << std::endl;
        k[1] = getPersonalDerivatives(targets, integrate(sv, k[0]*(dt/2)) );
        kf[1] = getRelativeDerivatives(targets, followers, *referenceFrame,
                                       integrate(sv, k[0]*(dt/2)), integrate(svf, kf[0]*(dt/2)) );
        
//        std::cout << "RK3" << std::endl;
        k[2] = getPersonalDerivatives(targets, integrate(sv, k[1]*(dt/2)) );
        kf[2] = getRelativeDerivatives(targets, followers, *referenceFrame,
                                       integrate(sv, k[1]*(dt/2)), integrate(svf, kf[1]*(dt/2)) );
        
//        std::cout << "RK4" << std::endl;
        k[3] = getPersonalDerivatives(targets, integrate(sv, k[2]*dt) );
        kf[3] = getRelativeDerivatives(targets, followers, *referenceFrame,
                                       integrate(sv, k[2]*dt), integrate(svf, kf[2]*dt) );
        
        dx = (k[0]+k[1]*2+k[2]*2+k[3])*(dt/6);
        dxf = (kf[0]+kf[1]*2+kf[2]*2+kf[3])*(dt/6);
        
//        for(size_t i=0; i<4; ++i)
//        {
//            std::cout << "k" << i+1 << ": ";
//            for(size_t j=0; j<kf[i].size(); ++j)
//                std::cout << kf[i][j].transpose() << " | ";
//            std::cout << "\n";
//        }
        
//        std::cout << "dx/dt: ";
//        for(size_t j=0; j<dxf.size(); ++j)
//            std::cout << dxf[j].transpose()/dt << " | ";
//        std::cout << "\n";
                
        sv = integrate(sv, dx);
        setStates(targets, sv);
        
//        std::cout << "Final\n";
//        for(size_t i=0; i<sv.size(); ++i)
//            std::cout << sv[i].x.matrix() << "\n" << svtest[i].v.transpose() << "\n";

        svf = integrate(svf, dxf);
        setStates(followers, svf);
        
//        std::cout << targets[1]->name() << "\t\taccel\t"
//                  << targets[1]->acceleration(*referenceFrame).transpose()
//                  << "\n";
        
//        std::cout << followers[1]->name() << "\t\taccel\t" 
//                  << followers[1]->acceleration(*referenceFrame).transpose() 
//                  << "\n";
        
        std::cout << targets[1]->name() << "\t\tvel\t" 
                  << targets[1]->velocity(*referenceFrame).transpose() 
                  << "\n";
        
        std::cout << followers[1]->name() << "\t\tvel\t" 
                  << followers[1]->velocity(*referenceFrame).transpose() 
                  << "\n";
        
//        Eigen::AngleAxisd aa(targets[1]->respectToWorld().rotation());
//        double theta = aa.angle();
//        std::cout << "2WxP:\t" 
//                  << -0.4*sin(theta)-0.4*cos(theta) << "\t" 
//                  <<  0.4*cos(theta)-0.4*sin(theta) << "\t" 
//                  << 0 << "\n";

        std::cout << std::endl;

        AkinNode::update();
    }

    Frame* referenceFrame;
    std::vector<Frame*> targets;
    std::vector<Frame*> followers;

    StateVector sv;
    StateVector svf;
    DerivVector k[4];
    DerivVector kf[4];
    DerivVector dx;
    DerivVector dxf;

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

    node->referenceFrame = &Frame::World();
//    Frame ref(Frame::World(), "Ref");
//    node->referenceFrame = &ref;
//    ref.respectToRef(Transform(Translation(0,1,1),Rotation(20*DEG,Axis(1,1,0))));
//    ref.relativeLinearVelocity(Velocity(0,0,-0.2));
//    ref.relativeAngularVelocity(Velocity(1,0.2,-0.3));

    Frame A(Frame::World(), "A");
    node->targets.push_back(&A);
    Frame B(Transform(Translation(0,0,0), Rotation()), A, "B");
    node->targets.push_back(&B);
//    Frame C(Transform(Translation(0,0.5,0), Rotation()), B, "C");
//    node->targets.push_back(&C);
//    Frame D(Transform(Translation(1,0,0), Rotation()), C, "D");
//    node->targets.push_back(&D);

    node->addRootFrame(A);
    A.relativeAngularVelocity(Velocity(0,0,1));
    B.relativeLinearVelocity(Velocity(0.2,0.2,0.05));
//    B.relativeAngularVelocity(Velocity(1,1,1));
//    C.relativeAngularVelocity(Velocity(0,1,0));
    
//    double scale = 1;
//    A.relativeAngularAcceleration(Acceleration(0,0,1)*scale);
//    B.relativeLinearAcceleration(Acceleration(1,1,0)/2*scale);
//    D.relativeLinearAcceleration(Acceleration(0.5,1,2)/4*scale);

    Frame Af(A.withRespectTo(*node->referenceFrame), *node->referenceFrame, "A follower");
    Af.relativeVelocity(A.velocity(Af.refFrame()));
    node->followers.push_back(&Af);
    Frame Bf(B.withRespectTo(*node->referenceFrame), *node->referenceFrame, "B follower");
    Bf.relativeVelocity(B.velocity(Bf.refFrame()));
    node->followers.push_back(&Bf);
//    Frame Cf(C.withRespectTo(*node->referenceFrame), *node->referenceFrame, "C follower");
//    Cf.relativeVelocity(C.velocity(Cf.refFrame()));
//    node->followers.push_back(&Cf);
//    Frame Df(D.withRespectTo(*node->referenceFrame), *node->referenceFrame, "D follower");
//    Df.relativeVelocity(D.velocity(Df.refFrame()));
//    node->followers.push_back(&Df);

    node->addRootFrame(*node->referenceFrame);
    
    if(!node->referenceFrame->isWorld())
    {
        node->targets.push_back(node->referenceFrame);
        Frame* Rf = new Frame(Transform::Identity(), *node->referenceFrame, "Ref follower");
        node->followers.push_back(Rf);
    }

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
