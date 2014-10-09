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

typedef enum {

    MASS = 0,

    FIRST_MOMENT_X,
    FIRST_MOMENT_Y,
    FIRST_MOMENT_Z,

    SECOND_MOMENT_XX,
    SECOND_MOMENT_XY,
    SECOND_MOMENT_XZ,

    SECOND_MOMENT_YY,
    SECOND_MOMENT_YZ,

    SECOND_MOMENT_ZZ

} inertia_param_t;

class StandardInertiaParameters;
class MinimalInertiaParameters;

class InertiaParameters : public std::vector< std::pair<inertia_param_t,double> >
{
public:

//    using std::vector< std::pair<inertia_param_t,double> >::vector;



};

class StandardInertiaParameters
{
public:

    double mass;
    Eigen::Vector3d centerOfMass;
    Eigen::Matrix3d inertiaTensor;

    InertiaParameters getParameters() const;

};

class MinimalInertiaParameters
{
public:

    InertiaParameters parameters;


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
