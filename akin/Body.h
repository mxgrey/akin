#ifndef AKIN_BODY_H
#define AKIN_BODY_H

#include "akin/Frame.h"
#include <map>

namespace akin {

typedef enum {

    EXPLICIT_EULER=0,

    NUM_INTEGRATION_METHODS

} integration_method_t;

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

    FEATHERSTONE=0,

    NUM_INVERSE_DYNAMICS_METHODS

} inverse_dynamics_method_t;

class InertiaBase
{
public:

    // This class is inherited by both Body and Robot so that either can
    // be attached to a manipulator using the same API.
    InertiaBase();

    friend class Manipulator;

    virtual Translation com(const Frame& withRespectToFrame = Frame::World()) const = 0;
    virtual double mass() const = 0;
    virtual Eigen::Matrix3d inertiaTensor(
            const Frame& withRespectToFrame = Frame::World()) const = 0;

    virtual FreeVector getForces(const Frame& withRepsectToFrame = Frame::World()) const = 0;
    virtual FreeVector getMoments(const Frame& withRespectToFrame = Frame::World()) const = 0;
    virtual Screw getWrench(const Frame& withRespectToFrame = Frame::World()) const = 0;
    
    virtual void setDynamicsMode(dynamics_mode_t mode);
    virtual dynamics_mode_t getDynamicsMode() const;

    virtual bool notifyDynUpdate();
    virtual bool needsDynUpdate() const;

    virtual const Matrix6d& _ABA_Ia() const;
    virtual const Vector6d& _ABA_pa() const;
    virtual const Vector6d& _ABA_c() const;
    virtual const Vector6d& _ABA_a() const;
    virtual const Vector6d& _ABA_arel() const;

    virtual const Matrix6Xd& _ABA_h() const;
    virtual const Eigen::VectorXd& _ABA_u() const;
    virtual const Eigen::MatrixXd& _ABA_D() const;
    virtual const Eigen::VectorXd& _ABA_qdd() const;

    integration_method_t integration_method;
    void integrate(double dt);
    virtual void integrate(integration_method_t method, double dt) = 0;

protected:

    virtual void _computeABA_pass2() const = 0;
    virtual void _computeABA_pass3() const = 0;

    mutable bool _needsDynUpdate;
    mutable bool _needsAbiUpdate;

    dynamics_mode_t _mode;
    InertiaBase* _attachment;

    mutable Matrix6d _Ia;               // pass2
    mutable Vector6d _pa;               // pass2
    mutable Vector6d _c;                // pass2
    mutable Vector6d _a;                // pass3
    mutable Vector6d _arel;             // pass3

    // TODO: Consider what to do for multiple degrees of freedom
    // Templates might be a possible solution
    mutable Matrix6Xd _h;               // 6 x DOF          pass2
    mutable Eigen::VectorXd _u;         // DOF x 1          pass2
    mutable Eigen::MatrixXd _D;         // DOF x DOF        pass2
    mutable Eigen::VectorXd _qdd;       // DOF x 1          pass3

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

typedef enum {

    INERTIA_WRT_COM,
    INERTIA_WRT_FRAME,

    NUM_INERTIA_TYPES

} inertia_type_t;

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

    StandardInertiaParameters();

    inertia_type_t type;

    double mass;
    Eigen::Vector3d centerOfMass;
    Eigen::Matrix3d tensor;

    void mirrorTheCurrentTensorValues(bool useUpperRight = true);
    void convertTo(inertia_type_t newType);

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
    inline virtual ~Body() { }

    Translation localCom() const;
    Translation com(const Frame& withRespectToFrame = Frame::World()) const;
    double mass() const;
    Eigen::Matrix3d inertiaTensor(const Frame& withRespectToFrame = Frame::World()) const;
    const StandardInertiaParameters& inertia() const;

    void mass(double newMass);
    void com(const Translation& newCoM);
    bool inertiaTensor(const Eigen::Matrix3d& newTensor,
                       inertia_type_t type = INERTIA_WRT_COM);
    void inertia(const StandardInertiaParameters& newInertia);

    virtual FreeVector getForces(const Frame &withRespectToFrame) const;
    virtual FreeVector getMoments(const Frame &withRespectToFrame) const;
    Screw getWrench(const Frame &withRespectToFrame) const;

    virtual void notifyPosUpdate();
    virtual void notifyVelUpdate();
    virtual void notifyAccUpdate();

    virtual const Acceleration& relativeLinearAcceleration() const;
    virtual const Acceleration& relativeAngularAcceleration() const;

    virtual void integrate(integration_method_t method, double dt);
    
protected:

    StandardInertiaParameters _inertia;
    void _recompute_Ia();

    virtual void _explicit_euler_integration(double dt);

    virtual void _computeABA_pass2() const;
    virtual void _computeABA_pass3() const;

    Eigen::Vector3d _sumForces_wrtWorld() const;
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
