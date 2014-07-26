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
    
    RobotConstraintBase(Robot* robot, const std::vector<size_t>& joints) :
        _robot(robot),
        _joints(joints) {  }
    
    bool changeRobot(Robot* newRobot, bool reconfigure=true) {
        
        _robot = newRobot;
        
        if(reconfigure)
            return _reconfigure();
        return false;
    }
    
    bool changeJoints(const std::vector<size_t>& newJoints, bool reconfigure=true) {
        
        _joints = newJoints;
        
        if(reconfigure)
            return _reconfigure();
        return false;
    }
    
    bool changeSetup(Robot* newRobot, const std::vector<size_t>& newJoints) {
        changeRobot(newRobot, false);
        return changeJoints(newJoints);
    }

protected:
    
    virtual bool _reconfigure() = 0;
    
    Robot* _robot;
    std::vector<size_t> _joints;
    
};

template<int Q, int W>
class RobotJacobianConstraint : public JacobianConstraint<Q,W>, public RobotConstraintBase
{
public:
    
    typedef typename Constraint<Q>::VectorQ VectorQ;
    
    RobotJacobianConstraint() : RobotConstraintBase() { }
    RobotJacobianConstraint(Robot* robot, const std::vector<size_t>& joints) :
        RobotConstraintBase(robot, joints) { }
    
    
protected:
    
    virtual void _update(const VectorQ& config){
        for(int i=0; i<Q; ++i)
            _robot->joint(_joints[i]).value(config[i]);
    }
    
};

template<int Q>
class ManipConstraint : public RobotJacobianConstraint<Q,6>
{
public:
    
    typedef typename Constraint<Q>::VectorQ VectorQ;
    typedef typename JacobianConstraint<Q,6>::Error Error;
    typedef typename JacobianConstraint<Q,6>::Jacobian Jacobian;
//    typedef Eigen::Matrix<double,6,Q> Jacobian;
    
    ManipConstraint() :
        RobotConstraintBase(), manip(NULL), target(Frame::World(), "manip_target")
    { _initializeDefaults(); }
    ManipConstraint(Robot* robot, const std::vector<size_t>& joints) :
        RobotJacobianConstraint<Q,6>(robot, joints), 
        manip(NULL),  target(Frame::World(), "manip_target") 
    { _initializeDefaults(); }
    ManipConstraint(const Manipulator* manipulator, Robot* robot, 
                    const std::vector<size_t> joints) :
        RobotJacobianConstraint<Q,6>(robot, joints),
        manip(manipulator), target(Frame::World(), manipulator->name()+"_target")
    { target.respectToRef(manip->respectToWorld()); _initializeDefaults(); }
    
    const Manipulator* manip;
    Frame target;
    Error min_limits;
    Error max_limits;
    
    virtual const Jacobian& getJacobian(const VectorQ& config, bool update=true) {
        if(update)
            _update(config);
        
        this->_Jacobian.setZero();
        for(size_t i=0; i<this->_joints.size(); ++i)
            if(_dependency[i])
                this->_Jacobian.template block<6,1>(0,i) = 
                        this->_robot->const_joint(this->_joints[i]).Jacobian(
                        manip->point(), target.refFrame(), false);

        return this->_Jacobian;
    }
    
    virtual const Error& getError(const VectorQ& config, 
                                  bool fromCenter=false, bool update=true) {
        if(update)
            _update(config);
        
        Transform tf_error = manip->withRespectTo(target.refFrame())*target.respectToRef().inverse();
        const Eigen::Vector3d& v = tf_error.translation();
        const Eigen::Matrix3d& rot = tf_error.rotation().matrix();
        
        for(size_t i=0; i<3; ++i)
            this->_error[i] = v[i];
        this->_error[3] =  atan2(rot(2,1), rot(2,2));
        this->_error[4] = -asin(rot(2,0));
        this->_error[5] =  atan2(rot(1,0), rot(0,0));
        for(int i=0; i<6; ++i)
        {
            // Translational Error
            if( this->_error[i] < min_limits[i] )
            {
                if(fromCenter)
                    this->_error[i] = this->_error[i] - (min_limits[i]+max_limits[i])/2.0;
                else
                    this->_error[i] = this->_error[i] - min_limits[i];
            }
            else if( max_limits[i] < this->_error[i] )
            {
                if(fromCenter)
                    this->_error[i] = this->_error[i] - (min_limits[i]+max_limits[i])/2.0;
                else
                    this->_error[i] = this->_error[i] - max_limits[i];
            }
            else
                this->_error[i] = 0;
        }
        
        if(this->error_weights.size() == Q)
            for(int i=0; i<Q; ++i)
                this->_error[i] *= this->error_weights[i];
        
        return this->_error;
    }
    
protected:
    
    void _initializeDefaults() {
         min_limits.setOnes(); max_limits.setOnes();
         min_limits *= 0.001;  max_limits *= 0.001;
         this->error_clamp = 0.2; this->component_clamp = 0.2;
         this->computeErrorFromCenter = true;
    }

    std::vector<bool> _dependency;
    bool _reconfigure() {
        if(this->_robot==NULL)
            return false;
        
        this->_robot->verb.Assert(this->_joints.size()==Q, verbosity::ASSERT_CRITICAL,
                            "ManipConstraint templated for "+std::to_string(Q)
                            +" DoF was given an index array of size "
                            +std::to_string(this->_joints.size())+"!");
        
        if(manip==NULL)
            return false;
        
        _dependency.clear();
        for(int i=0; i<Q; ++i)
        {
            const Joint& j = this->_robot->const_joint(this->_joints[i]);
            if(manip->descendsFrom(j.const_childLink()))
                _dependency.push_back(true);
            else
                _dependency.push_back(false);
        }
        
        return true;
    }
};



} // namespace akin

#endif // AKIN_ROBOTCONSTRAINT_H
