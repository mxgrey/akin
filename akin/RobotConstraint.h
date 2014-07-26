#ifndef AKIN_ROBOTCONSTRAINT_H
#define AKIN_ROBOTCONSTRAINT_H

#include "Constraint.h"
#include "Robot.h"

namespace akin {

class RobotConstraintBase
{
public:
    
    RobotConstraintBase() :
        _robot(NULL) { }
    
    RobotConstraintBase(const Robot* robot, const std::vector<size_t> joints) :
        _robot(robot),
        _joints(joints) { _reconfigure(); }
    
    bool changeRobot(const Robot* newRobot, bool reconfigure=true) {
        
        _robot = newRobot;
        
        if(reconfigure)
            return _reconfigure();
        return false;
    }
    
    bool changeJoints(const std::vector<size_t> newJoints, bool reconfigure=true) {
        
        _joints = newJoints;
        
        if(reconfigure)
            return _reconfigure();
        return false;
    }
    
    bool changeSetup(const Robot* newRobot, const std::vector<size_t> newJoints) {
        changeRobot(newRobot, false);
        return changeJoints(newJoints);
    }

protected:
    
    virtual bool _reconfigure() = 0;
    
    const Robot* _robot;
    std::vector<size_t> _joints;
    
};

template<int N>
class ManipConstraint : public Constraint<N>, public RobotConstraintBase
{
public:
    
    typedef Eigen::Matrix<double,6,N> Jacobian;
    typedef Eigen::Matrix<double,6,1> Error;
    
    ManipConstraint() :
        RobotConstraintBase(), manip(NULL), target(Frame::World(), "manip_target")
    { _initializeDefaults(); }
    ManipConstraint(const Robot* robot, const std::vector<size_t> joints) :
        RobotConstraintBase(robot, joints), 
        manip(NULL),  target(Frame::World(), "manip_target") 
    { _initializeDefaults(); }
    ManipConstraint(const Manipulator* manipulator, const Robot* robot, 
                    const std::vector<size_t> joints) :
        RobotConstraintBase(robot, joints),
        manip(manipulator), target(Frame::World(), manipulator->name()+"_target")
    { target.respectToRef(manip->respectToWorld()); _initializeDefaults(); }
    
    const Manipulator* manip;
    Frame target;
    Error min_limits;
    Error max_limits;
    bool computeErrorFromCenter;
    
    const Jacobian& getJacobian(const VectorN& config, bool update=true) const {
        if(update)
            _update(config);
        
        _Jacobian.setZero();
        for(size_t i=0; i<_joints.size(); ++i)
            if(_dependency[i])
                _Jacobian.block<6,1>(0,i) = _robot->joint(_joints[i]).Jacobian(
                                                manip->point(), target.refFrame(), false);
        return _Jacobian;
    }
    
    const Error& getError(const VectorN& config, bool fromCenter=false, bool update=true) const {
        if(update)
            _update(config);
        
        Transform tf_error = manip->withRespectTo(target.refFrame())*target.respectToRef().inverse();
        
        const Eigen::Vector3d& v = tf_error.translation();
        const Eigen::Matrix3d& rot = tf_error.rotation().matrix();
        for(size_t i=0; i<3; ++i)
            _error[i] = v[i];
        _error[3] =  atan2(rot(2,1), rot(2,2));
        _error[4] = -asin(rot(2,0));
        _error[5] =  atan2(rot(1,0), rot(0,0));
        for(size_t i=0; i<6; ++i)
        {
            // Translational Error
            if( _error[i] < min_limits[i] )
            {
                if(fromCenter)
                    _error[i] = v[i] - (min_limits[i]+max_limits[i])/2.0;
                else
                    _error[i] = v[i] - min_limits[i];
            }
            else if( max_limits[i] < _error[i] )
            {
                if(fromCenter)
                    _error[i] = v[i] - (min_limits[i]+max_limits[i])/2.0;
                else
                    _error[i] = v[i] - max_limits[i];
            }
            else
                _error[i] = 0;
        }
        
        if(error_weights.size() == N)
            for(int i=0; i<N; ++i)
                _error *= error_weights[i];
        
        return _error;
    }
    
    virtual Validity getGradient(VectorN &gradient, const VectorN &configuration) {
        _update(config);
        
        getError(config, computeErrorFromCenter, false);
        
        
    }
    
protected:
    
    void _initializeDefaults() {
         min_limits.setZero(); max_limits.setZero();
         computeErrorFromCenter = true;
    }
    
    void _update(const VectorN& config){
        for(int i=0; i<N; ++i)
            _robot->joint(_joints[i]).value(config[i]);
    }
    
    Validity _computeCurrentValidity() {
        Validity v;
        
        return v;
    }

    Jacobian _Jacobian;
    Error _error;
    Error _center;
    std::vector<bool> _dependency;
    bool _reconfigure() {
        if(_robot==NULL)
            return false;
        
        _robot->verb.Assert(_joints.size()==N, verbosity::ASSERT_CRITICAL,
                            "ManipConstraint templated for "+std::to_string(N)
                            +" DoF was given an index array of size "
                            +std::to_string(_joints.size())+"!");
        
        if(manip==NULL)
            return false;
        
        _dependency.clear();
        for(int i=0; i<N; ++i)
        {
            const Joint& j = _robot->joint(_joints[i]);
            if(manip->descendsFrom(j.childLink()))
                _dependency.push_back(true);
            else
                _dependency.push_back(false);
        }
    }
};

typedef ManipConstraint TaskSpaceRegion;


} // namespace akin

#endif // AKIN_ROBOTCONSTRAINT_H
