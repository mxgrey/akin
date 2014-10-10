
#include "akin/Body.h"

using namespace akin;
using namespace std;

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
