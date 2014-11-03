
#include "akin/Body.h"

using namespace akin;
using namespace std;

InertiaBase::InertiaBase() :
    _needsDynUpdate(true),
    _needsAbiUpdate(true),
    _mode(FORWARD),
    _attachment(NULL)
{
    
}

void InertiaBase::setDynamicsMode(dynamics_mode_t mode)
{
    if(_mode==mode)
        return;

    _mode = mode;
    if(_attachment)
        _attachment->setDynamicsMode(mode);
}

dynamics_mode_t InertiaBase::getDynamicsMode() const
{
    return _mode;
}

bool InertiaBase::notifyDynUpdate()
{
    if(_needsDynUpdate && (INVERSE==_mode || _needsAbiUpdate))
        return false;
    
    _needsDynUpdate = true;
    _needsAbiUpdate = true;
    if(_attachment)
        _attachment->notifyDynUpdate();

    return true;
}

bool InertiaBase::needsDynUpdate() const
{
    return _needsDynUpdate;
}

const Matrix6d& InertiaBase::_ABA_Ia() const
{
    if(_needsAbiUpdate)
        _computeABA_pass2();

    return _Ia;
}

const Vector6d& InertiaBase::_ABA_pa() const
{
    if(_needsAbiUpdate)
        _computeABA_pass2();

    return _pa;
}

const Vector6d& InertiaBase::_ABA_c() const
{
    if(_needsAbiUpdate)
        _computeABA_pass2();

    return _c;
}

const Vector6d& InertiaBase::_ABA_a() const
{
    if(_needsDynUpdate)
        _computeABA_pass3();

    return _a;
}

const Vector6d& InertiaBase::_ABA_arel() const
{
    if(_needsDynUpdate)
        _computeABA_pass3();

    return _arel;
}

const Matrix6Xd& InertiaBase::_ABA_h() const
{
    if(_needsAbiUpdate)
        _computeABA_pass2();

    return _h;
}

const Eigen::VectorXd& InertiaBase::_ABA_u() const
{
    if(_needsAbiUpdate)
        _computeABA_pass2();

    return _u;
}

const Eigen::MatrixXd& InertiaBase::_ABA_D() const
{
    if(_needsAbiUpdate)
        _computeABA_pass2();

    return _D;
}

const Eigen::VectorXd& InertiaBase::_ABA_qdd() const
{
    if(_needsDynUpdate)
        _computeABA_pass3();

    return _qdd;
}

void InertiaBase::integrate(double dt)
{
    integrate(integration_method, dt);
}

std::string akin::inertia_param_to_string(size_t param)
{
    switch (param) {
        case MASS:
            return "MASS";
        case FIRST_MOMENT_X:
            return "FIRST_MOMENT_X";
        case FIRST_MOMENT_Y:
            return "FIRST_MOMENT_Y";
        case FIRST_MOMENT_Z:
            return "FIRST_MOMENT_Z";
        case SECOND_MOMENT_XX:
            return "SECOND_MOMENT_XX";
        case SECOND_MOMENT_XY:
            return "SECOND_MOMENT_XY";
        case SECOND_MOMENT_XZ:
            return "SECOND_MOMENT_XZ";
        case SECOND_MOMENT_YY:
            return "SECOND_MOMENT_YY";
        case SECOND_MOMENT_YZ:
            return "SECOND_MOMENT_YZ";
        case SECOND_MOMENT_ZZ:
            return "SECOND_MOMENT_ZZ";
        case NUM_INERTIA_PARAMS:
        default:
            return "INVALID_INERTIA_PARAM";
    }
}

InertiaParameters::InertiaParameters() { }

InertiaParameters::InertiaParameters(const StandardInertiaParameters& parameters)
{
    *this = parameters.getParameters();
}

InertiaParameters::InertiaParameters(const MinimalInertiaParameters& parameters)
{
    *this = parameters.parameters;
}

InertiaParameters StandardInertiaParameters::getParameters() const
{
    InertiaParameters result;
    result.resize(NUM_INERTIA_PARAMS);
    
    result[0] = InertiaValue(MASS, mass);
    
    const Eigen::Vector3d& first_moment = centerOfMass*mass;
    result[1] = InertiaValue(FIRST_MOMENT_X, first_moment[0]);
    result[2] = InertiaValue(FIRST_MOMENT_Y, first_moment[1]);
    result[3] = InertiaValue(FIRST_MOMENT_Z, first_moment[2]);
    
    
    result[4] = InertiaValue(SECOND_MOMENT_XX, inertiaTensor(0,0));
    result[5] = InertiaValue(SECOND_MOMENT_XY, inertiaTensor(0,1));
    result[6] = InertiaValue(SECOND_MOMENT_XZ, inertiaTensor(0,2));
    
    result[7] = InertiaValue(SECOND_MOMENT_YY, inertiaTensor(1,1));
    result[8] = InertiaValue(SECOND_MOMENT_YZ, inertiaTensor(1,2));
    
    result[9] = InertiaValue(SECOND_MOMENT_ZZ, inertiaTensor(2,2));
    
    return result;
}

static bool checkCount(size_t required_count, size_t actual_count, const std::string& name)
{
    if(0 < actual_count && actual_count < required_count)
    {
        std::cout << "Only " << actual_count << " values were provided for " << name << ","
                     " but " << required_count << " are required!\n";
        return false;
    }
    else if(actual_count > required_count)
    {
        std::cout << "Too many values were provided for the first moment of inertia: " 
                  << actual_count << ". This must be exactly " << required_count << "!\n";
        return false;
    }
    
    return true;
}

bool StandardInertiaParameters::setParameters(const InertiaParameters& parameters)
{
    size_t fm_count = 0;
    Eigen::Vector3d first_moment;
    size_t sm_count = 0;
    Eigen::Matrix3d second_moment;
    for(size_t i=0; i<parameters.size(); ++i)
    {
        size_t type = parameters[i].first;
        double value = parameters[i].second;
        
        switch (type) {
            case MASS:
                mass = value;
                break;
            case FIRST_MOMENT_X:
                first_moment[0] = value;
                ++fm_count;
                break;
            case FIRST_MOMENT_Y:
                first_moment[1] = value;
                ++fm_count;
                break;
            case FIRST_MOMENT_Z:
                first_moment[2] = value;
                ++fm_count;
                break;
            case SECOND_MOMENT_XX:
                second_moment(0,0) = value;
                ++sm_count;
                break;
            case SECOND_MOMENT_XY:
                second_moment(0,1) = value;
                second_moment(1,0) = value;
                ++sm_count;
                break;
            case SECOND_MOMENT_XZ:
                second_moment(0,2) = value;
                second_moment(2,0) = value;
                ++sm_count;
                break;
            case SECOND_MOMENT_YY:
                second_moment(1,1) = value;
                ++sm_count;
                break;
            case SECOND_MOMENT_YZ:
                second_moment(1,2) = value;
                second_moment(2,1) = value;
                ++sm_count;
                break;
            case SECOND_MOMENT_ZZ:
                second_moment(2,2) = value;
                ++sm_count;
                break;
            default:
                std::cout << "Invalid inertia parameter type passed into setParameters(~): " 
                          << type << "\n";
                break;
        }
    }
    
    bool valid = true;
    
    valid &= checkCount(3, fm_count, "the first moment of inertia");
    if(fm_count==3)
        centerOfMass = first_moment/mass;
    
    valid &= checkCount(6, sm_count, "the second moment of inertia");
    if(sm_count==6)
        inertiaTensor = second_moment;
    
    return valid;
}



Body::Body(Frame &referenceFrame, const string &bodyName) :
    Frame(referenceFrame, bodyName),
    mass(0),
    com(*this, bodyName+"_com")
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
        return _sumForces_wrtWorld();

//    if(this == &withRespectToFrame)
//        return respectToWorld().rotation().transpose()*_sumForces_wrtWorld();

    const Eigen::Isometry3d& wrt = withRespectToFrame.respectToWorld();

    return wrt.rotation().transpose()*_sumForces_wrtWorld();
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

void Body::notifyPosUpdate()
{
    notifyDynUpdate();
    Frame::notifyPosUpdate();
}

void Body::notifyVelUpdate()
{
    notifyDynUpdate();
    Frame::notifyVelUpdate();
}

void Body::notifyAccUpdate()
{
    if(INVERSE==_mode)
        notifyDynUpdate();
    Frame::notifyAccUpdate();
}

const Acceleration& Body::relativeLinearAcceleration() const
{
    if(INVERSE==_mode)
        return Frame::relativeLinearAcceleration();

    if(_needsDynUpdate)
        _computeABA_pass3();

    return _relativeLinearAcc;
}

const Acceleration& Body::relativeAngularAcceleration() const
{
    if(INVERSE==_mode)
        return Frame::relativeAngularAcceleration();

    if(_needsDynUpdate)
        _computeABA_pass3();

    return _relativeAngularAcc;
}

void Body::integrate(integration_method_t method, double dt)
{
    switch(method)
    {
        case EXPLICIT_EULER: return _explicit_euler_integration(dt);
        default:
            verb.Assert(false, verbosity::ASSERT_CASUAL,
                        "Calling integrate(~) method on a Body with an invalid method ("
                        +to_string(method)+")");
    }
}

void Body::_explicit_euler_integration(double dt)
{
    const Transform& tf0 = respectToRef();
    Transform tf;

    tf.translate(tf0.translation()+relativeLinearVelocity()*dt);
    tf.rotate(Rotation(FreeVector(relativeAngularVelocity()*dt)));
    tf.rotate(tf0.rotation());

    const Acceleration& a = relativeLinearAcceleration();
    const Acceleration& alpha = relativeAngularAcceleration();

    relativeLinearVelocity(a*dt);
    relativeAngularVelocity(alpha*dt);

    respectToRef(tf);
}

void Body::_computeABA_pass2() const
{
    // TODO: Consider computing this when new inertial parameters are passed in instead of
    // computing it each time we pass through
    _Ia.block<3,3>(0,0) = _inertiaTensor_wrtLocalFrame;
    _Ia.block<3,3>(0,3) = mass*skew(com.respectToRef());
    _Ia.block<3,3>(3,0) = -_Ia.block<3,3>(0,3);
    _Ia.block<3,3>(3,3) = mass*Eigen::Matrix3d::Identity();

    Spatial v;
    v.upper() = respectToWorld().rotation()*angularVelocity();
    v.lower() = respectToWorld().rotation()*linearVelocity();

    _pa.block<3,1>(0,0) = v.upper().cross(_Ia.block<3,3>(0,0)*v.upper())
            + mass*com.cross(v.upper().cross(v.lower()));
    _pa.block<3,1>(3,0) = mass*v.upper().cross(v.upper().cross(com))
            + mass*v.upper().cross(v.lower());

    if(_attachment)
    {
        // TODO: Investigate whether it is valid to ignore h, d, and u for the attached case.
        // If attached objects are permitted some degrees of freedom, we will definitely need
        // to do some computations here.
        _c.setZero();
    }
    else
    {
        _c.block<3,1>(0,0) = v.upper().cross(relativeAngularVelocity());
        _c.block<3,1>(3,0) = v.lower().cross(relativeAngularVelocity())
                + v.upper().cross(relativeLinearVelocity());
        _h = _Ia;
        _D = _h;
        Vector6d F;
        F.block<3,1>(0,0) = _appliedMoments_wrtWorld;
        F.block<3,1>(3,0) = _sumForces_wrtWorld();
        _u = F - _pa;
    }

    _needsAbiUpdate = false;
}

void Body::_computeABA_pass3() const
{
    // TODO: This is all horribly wrong
    if(_attachment)
    {
        // TODO: Investigate whether I am handling the "weld joint" case correctly.
        // Later I should add the possibility of an object sliding around in the grip.
        const Matrix6d& X = spatial_transform(respectToRef());
        _a = X*_attachment->_ABA_a() + _ABA_c();
        _qdd.setZero();
        _arel.setZero();
    }
    else
    {
        const Matrix6d& X = spatial_transform(respectToRef());
        Vector6d a_ref;
        a_ref.block<3,1>(0,0) = refFrame().respectToWorld().rotation().transpose()*
                refFrame().angularAcceleration();
        a_ref.block<3,1>(3,0) = refFrame().respectToWorld().rotation().transpose()*(
                    refFrame().linearAcceleration()
                    - refFrame().angularVelocity().cross(refFrame().linearVelocity()) );

        _a = X*a_ref + _ABA_c();
        _arel = _ABA_D().inverse()*(_ABA_u() - _ABA_h().transpose()*_a);
        _qdd = _arel;
        _a = _a + _arel;
    }

    _relativeAngularAcc = _arel.block<3,1>(0,0);
    _relativeLinearAcc = _arel.block<3,1>(3,0) + respectToWorld().rotation().transpose()*(
                angularVelocity().cross(linearVelocity()));

    _needsDynUpdate = false;
}

Eigen::Vector3d Body::_sumForces_wrtWorld() const
{
    return _appliedForces_wrtWorld+mass*_gravity.respectToWorld();
}

std::ostream& operator<<(std::ostream& stream, const akin::Body& someBody)
{
    std::cout << "Body named '" << someBody.name() << "' has mass " << someBody.mass 
              << " and a relative Center of Mass <" << someBody.com.transpose() << ">\n";
    stream << (akin::Frame&)someBody;
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const akin::InertiaParameters& params)
{
    for(size_t i=0; i<params.size(); ++i)
    {
        size_t type = params[i].first%NUM_INERTIA_PARAMS;
        size_t body = params[i].first/NUM_INERTIA_PARAMS;
        
        stream << inertia_param_to_string(type) 
               << " (" << body << "): " 
               << params[i].second
               << "\n";
    }
    
    return stream;
}

static void stream_parameter_group(std::ostream& stream, const std::vector<size_t>& grouped)
{
    for(size_t j=0; j<grouped.size(); ++j)
    {
        size_t type = grouped[j]%NUM_INERTIA_PARAMS;
        size_t body = grouped[j]/NUM_INERTIA_PARAMS;
        stream << inertia_param_to_string(type) 
               << " (" << body << ")";
        if(j < grouped.size()-1)
            stream << ", ";
    }
}

std::ostream& operator<<(std::ostream& stream, const akin::InertiaGrouping& grouping)
{
    for(InertiaGrouping::const_iterator i = grouping.begin(), end = grouping.end(); i != end; ++i)
    {
        size_t base = i->first;
        
        stream << inertia_param_to_string(base%NUM_INERTIA_PARAMS) 
               << " (" << base/NUM_INERTIA_PARAMS << ") is grouped with: ";
        
        stream_parameter_group(stream, i->second);
        stream << "\n";
    }
    
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const akin::StandardInertiaParameters& standard)
{
    stream << "Mass: " << standard.mass << "\n";
    stream << "Center of Mass: " << standard.centerOfMass << "\n";
    stream << "(First Moment: " << standard.mass*standard.centerOfMass << ")\n";
    stream << "Inertia Tensor:\n" << standard.inertiaTensor << "\n";
    
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const akin::MinimalInertiaParameters& minimal)
{
    if(minimal.parameters.size() != minimal.grouping.size())
    {
        stream << "Warning: Minimal Inertial Parameter set is malformed: parameter count (" 
               << minimal.parameters.size() << ") does not match grouping count (" 
               << minimal.grouping.size() << ")\n";
    }
    
    for(size_t i=0; i<minimal.parameters.size(); ++i)
    {
        size_t base = minimal.parameters[i].first;
        
        stream << inertia_param_to_string(base%NUM_INERTIA_PARAMS) 
               << " (" << base/NUM_INERTIA_PARAMS << ") is " 
               << minimal.parameters[i].second;
        
        InertiaGrouping::const_iterator G = minimal.grouping.find(base);
        if(G == minimal.grouping.end())
        {
            stream << " <Warning: The grouping for this parameter (#" << base 
                   << ") could not be found!>";
        }
        else
        {
            stream << " and is grouped with: ";
            stream_parameter_group(stream, G->second);
        }
        
        stream << "\n";
    }
    
    return stream;
}
