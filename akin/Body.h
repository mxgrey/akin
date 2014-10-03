#ifndef AKIN_BODY_H
#define AKIN_BODY_H

#include "akin/Frame.h"

namespace akin {

class InertiaBase
{
public:

    // Fill with pure abstract functions like getCom(), getMass(), getForces()
    //
    // This class will be inherited by both Body and Robot so that either can
    // be attached to a manipulator using the same API.

    virtual Translation getCom(const Frame& withRespectToFrame = Frame::World()) const = 0;
    virtual double getMass() const = 0;
    virtual Eigen::Matrix3d getInertiaTensor(
            const Frame& withRespectToFrame = Frame::World()) const = 0;

    virtual FreeVector getForces(const Frame& withRepsectToFrame = Frame::World()) const = 0;
    virtual FreeVector getMoments(const Frame& withRespectToFrame = Frame::World()) const = 0;
    virtual Screw getWrench(const Frame& withRespectToFrame = Frame::World()) const = 0;

};

class Body : public Frame, public InertiaBase
{
public:
    
    Body(Frame& referenceFrame, const std::string& bodyName);
    
    KinTranslation com;
    double mass;

    Translation getCom(const Frame& withRespectToFrame = Frame::World()) const;
    double getMass() const;
    virtual Eigen::Matrix3d getInertiaTensor(const Frame &withRespectToFrame) const;

    virtual FreeVector getForces(const Frame &withRespectToFrame) const;
    virtual FreeVector getMoments(const Frame &withRespectToFrame) const;
    Screw getWrench(const Frame &withRespectToFrame) const;
    
protected:

    Eigen::Matrix3d _inertiaTensor_wrtLocalFrame;
    FreeVector _appliedForces_wrtWorld;
    FreeVector _appliedMoments_wrtWorld;
    
};

typedef std::vector<Body*> BodyPtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, const akin::Body& someBody);

#endif // AKIN_BODY_H
