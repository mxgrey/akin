#ifndef AKIN_BODY_H
#define AKIN_BODY_H

#include "akin/Frame.h"
#include <map>

namespace akin {

typedef enum {
    
    // TODO: Think carefully how to name these enums
    FORWARD=0,
    INVERSE,
    
    NUM_DYNAMICS_MODES
    
} dynamics_mode_t;

typedef enum {
    
    STANDARD_NEWTON_EULER=0,
    MINIMAL_NEWTON_EULER,

    NUM_FORWARD_DYNAMICS_METHODS
    
} forward_dynamics_method_t;

typedef enum {

    STANDARD_LAGRANGIAN=0,

    NUM_INVERSE_DYNAMICS_METHODS

} inverse_dynamics_method_t;

class InertiaBase
{
public:

    // Fill with pure abstract functions like getCom(), getMass(), getForces()
    //
    // This class will be inherited by both Body and Robot so that either can
    // be attached to a manipulator using the same API.
    InertiaBase();

    virtual Translation getCom(const Frame& withRespectToFrame = Frame::World()) const = 0;
    virtual double getMass() const = 0;
    virtual Eigen::Matrix3d getInertiaTensor(
            const Frame& withRespectToFrame = Frame::World()) const = 0;

    virtual FreeVector getForces(const Frame& withRepsectToFrame = Frame::World()) const = 0;
    virtual FreeVector getMoments(const Frame& withRespectToFrame = Frame::World()) const = 0;
    virtual Screw getWrench(const Frame& withRespectToFrame = Frame::World()) const = 0;
    
    virtual void setDynamicsMode(dynamics_mode_t mode);
    
    virtual void notifyDynUpdate();
    
protected:
    
    bool _needsDynUpdate;
    dynamics_mode_t _mode;
    InertiaBase* _attachingPoint;

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

    SECOND_MOMENT_ZZ,
    
    NUM_INERTIA_PARAMS

} inertia_param_t;

std::string inertia_param_to_string(size_t param);

class StandardInertiaParameters;
class MinimalInertiaParameters;

typedef std::pair<size_t,double> InertiaValue;

class InertiaParameters : public std::vector<InertiaValue>
{
public:

    // TODO: Add this once C++11 functionality is attainable
//    using std::vector< std::pair<inertia_param_t,double> >::vector;
    
    InertiaParameters();
    InertiaParameters(const StandardInertiaParameters& parameters);
    InertiaParameters(const MinimalInertiaParameters& parameters);

};

class StandardInertiaParameters
{
public:

    double mass;
    Eigen::Vector3d centerOfMass;
    Eigen::Matrix3d inertiaTensor;

    InertiaParameters getParameters() const;
    bool setParameters(const InertiaParameters& parameters);

};

typedef std::map<size_t, std::vector<size_t> > InertiaGrouping;

class MinimalInertiaParameters
{
public:

    InertiaParameters parameters;
    InertiaGrouping grouping;

};

class Body : public Frame, public InertiaBase
{
public:
    
    Body(Frame& referenceFrame, const std::string& bodyName);
    
    double mass;
    KinTranslation com;

    Translation getCom(const Frame& withRespectToFrame = Frame::World()) const;
    double getMass() const;
    virtual Eigen::Matrix3d getInertiaTensor(const Frame &withRespectToFrame) const;

    virtual FreeVector getForces(const Frame &withRespectToFrame) const;
    virtual FreeVector getMoments(const Frame &withRespectToFrame) const;
    Screw getWrench(const Frame &withRespectToFrame) const;
    
protected:

    Eigen::Vector3d _sumForces_wrtWorld() const;
    Eigen::Matrix3d _inertiaTensor_wrtLocalFrame;
    FreeVector _appliedForces_wrtWorld;
    FreeVector _appliedMoments_wrtWorld;
    
};

typedef std::vector<Body*> BodyPtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& stream, const akin::Body& someBody);

std::ostream& operator<<(std::ostream& stream, const akin::InertiaParameters& params);
std::ostream& operator<<(std::ostream& stream, const akin::InertiaGrouping& grouping);

std::ostream& operator<<(std::ostream& stream, const akin::StandardInertiaParameters& standard);
std::ostream& operator<<(std::ostream& stream, const akin::MinimalInertiaParameters& minimal);

#endif // AKIN_BODY_H
