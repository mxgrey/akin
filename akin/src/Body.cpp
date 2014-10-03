
#include "akin/Body.h"

using namespace akin;
using namespace std;

Body::Body(Frame &referenceFrame, const string &bodyName) :
    Frame(referenceFrame, bodyName),
    com(*this, bodyName+"_com"),
    mass(0)
{
    com.setZero();
}

Translation Body::getCom(const Frame &withRespectToFrame) const
{
    return com.withRespectTo(withRespectToFrame);
}

double Body::getMass() const
{
    return mass;
}

Eigen::Matrix3d Body::getInertiaTensor(const Frame &withRespectToFrame) const
{
    if(this == &withRespectToFrame)
        return _inertiaTensor_wrtLocalFrame;

    if(withRespectToFrame.isWorld())
        return respectToWorld().rotation()
                *_inertiaTensor_wrtLocalFrame
                *respectToWorld().rotation().transpose();

    const Eigen::Isometry3d& wrt = withRespectTo(withRespectToFrame);

    return wrt.rotation()*_inertiaTensor_wrtLocalFrame*wrt.rotation().transpose();
}

FreeVector Body::getForces(const Frame& withRespectToFrame) const
{
    if(withRespectToFrame.isWorld())
        return _appliedForces_wrtWorld;

    if(this == &withRespectToFrame)
        return respectToWorld().rotation().transpose()*_appliedForces_wrtWorld;

    const Eigen::Isometry3d& wrt = withRespectToFrame.respectToWorld();

    return wrt.rotation().transpose()*_appliedForces_wrtWorld;
}

FreeVector Body::getMoments(const Frame &withRespectToFrame) const
{
    if(withRespectToFrame.isWorld())
        return _appliedMoments_wrtWorld;

    if(this == &withRespectToFrame)
        return respectToWorld().rotation().transpose()*_appliedMoments_wrtWorld;

    const Eigen::Isometry3d& wrt = withRespectToFrame.respectToWorld();

    return wrt.rotation().transpose()*_appliedMoments_wrtWorld;
}

Screw Body::getWrench(const Frame &withRespectToFrame) const
{
    return Screw(getForces(withRespectToFrame),getMoments(withRespectToFrame));
}

std::ostream& operator<<(std::ostream& oStrStream, const akin::Body& someBody)
{
    std::cout << "Body named '" << someBody.name() << "' has mass " << someBody.mass 
              << " and a relative Center of Mass <" << someBody.com.transpose() << ">\n";
    oStrStream << (akin::Frame&)someBody;
    return oStrStream;
}
