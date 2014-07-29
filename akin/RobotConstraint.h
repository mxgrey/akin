#ifndef AKIN_ROBOTCONSTRAINT_H
#define AKIN_ROBOTCONSTRAINT_H

#include "Constraint.h"
#include "Robot.h"

namespace akin {

class RobotConstraintBase : public virtual ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~RobotConstraintBase();
    
    RobotConstraintBase();
    RobotConstraintBase(Robot& robot, const std::vector<size_t>& joints);
    
    bool changeRobot(Robot* newRobot, bool reconfigure=true);
    bool changeJoints(const std::vector<size_t>& newJoints, bool reconfigure=true);
    bool changeSetup(Robot* newRobot, const std::vector<size_t>& newJoints);
    
    const std::vector<size_t>& getJoints() const;
    Robot* getRobot();

protected:
    
    virtual bool _reconfigure() = 0;
    
    Robot* _robot;
    std::vector<size_t> _joints;
    
};

template<int Q, int W>
class RobotJacobianConstraint : public JacobianConstraint<Q,W>, public virtual RobotConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~RobotJacobianConstraint() { }
    
    typedef typename Constraint<Q>::VectorQ VectorQ;
    RobotJacobianConstraint() { }
    RobotJacobianConstraint(int cspace_size) : JacobianConstraint<Q,W>(cspace_size) { }
    
protected:
    
    virtual void _update(const VectorQ& config){
        for(int i=0; i<this->_config_size; ++i)
            _robot->joint(_joints[i]).value(config[i]);
    }
    
};

class ManipConstraintBase : public virtual RobotConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~ManipConstraintBase();
    
    ManipConstraintBase();
    ManipConstraintBase(Manipulator& manipulator);
    
    Manipulator* manip();
    bool changeManipulator(Manipulator* manip);
    Frame target;
    Screw min_limits;
    Screw max_limits;
    
    virtual bool checkSetup() const;
    
protected:
    
    Manipulator* _manip;
    std::vector<bool> _dependency;
    bool _reconfigure();
    
};

class NullManipConstraint : public ManipConstraintBase, public NullConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
};

template<int Q>
class ManipConstraint : public RobotJacobianConstraint<Q,6>, public ManipConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~ManipConstraint() { }
    
    typedef typename Constraint<Q>::VectorQ VectorQ;
    typedef typename JacobianConstraint<Q,6>::Error Error;
    typedef typename JacobianConstraint<Q,6>::Jacobian Jacobian;
    
    ManipConstraint() :
        RobotConstraintBase(), ManipConstraintBase() 
    { _initializeDefaults(); }
    ManipConstraint(int cspace_size) :
        RobotConstraintBase(), RobotJacobianConstraint<Q,6>(cspace_size), ManipConstraintBase()
    { _initializeDefaults(); }
    ManipConstraint(Manipulator& manipulator, const std::vector<size_t> joints):
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator)
    { _initializeDefaults(); }
    ManipConstraint(int cspace_size, Manipulator &manipulator, const std::vector<size_t> joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), 
        RobotJacobianConstraint<Q,6>(cspace_size), ManipConstraintBase(manipulator)
    { _initializeDefaults(); }
    
    virtual const Jacobian& getJacobian(const VectorQ& config, bool update=true) {
        if(!checkSetup()) { this->_Jacobian.setZero(); return this->_Jacobian; }
        
        if(update)
            _update(config);
        
        for(size_t i=0; i<this->_joints.size(); ++i)
        {
            if(_dependency[i])
                this->_Jacobian.template block<6,1>(0,i) = 
                        this->_robot->const_joint(this->_joints[i]).Jacobian(
                        _manip->point(), target.refFrame(), false);
            else
                this->_Jacobian.template block<6,1>(0,i) = Error::Zero();
        }

        return this->_Jacobian;
    }
    
    virtual const Error& getError(const VectorQ& config, 
                                  bool fromCenter=false, bool update=true) {
        if(!checkSetup()) { this->_error.setZero(); return this->_error; }

        if(update)
            _update(config);
        
        Transform tf_error = _manip->withRespectTo(target.refFrame())*target.respectToRef().inverse();
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
         min_limits *= -0.001;  max_limits *= 0.001;
         this->error_clamp = 0.2; this->component_clamp = 0.2;
         this->computeErrorFromCenter = true;
         error_weights.resize(6); error_weights.setOnes();
         error_weights[3] = 0.1; error_weights[4] = 0.1; error_weights[5] = 0.1;
         _reconfigure();
    }
    
};

typedef ManipConstraint<Eigen::Dynamic> ManipConstraintX;

} // namespace akin

#endif // AKIN_ROBOTCONSTRAINT_H
