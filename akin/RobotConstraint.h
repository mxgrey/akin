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
    
    virtual bool checkSetup() const;
    
    const std::vector<size_t>& getJoints() const;
    Robot* getRobot();

protected:
    
    virtual bool _reconfigure();
    bool _configured;
    
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
    KinTransform target;
    Screw min_limits;
    Screw max_limits;
    
    virtual bool checkSetup() const;
    
protected:
    
    Manipulator* _manip;
    std::vector<bool> _dependency;
    virtual bool _reconfigure();
    
};

class NullManipConstraint : public ManipConstraintBase, public NullConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NullManipConstraint();
    NullManipConstraint(Robot& robot);
};

template<int Q>
class ManipConstraint : public RobotJacobianConstraint<Q,6>, public virtual ManipConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~ManipConstraint() { }
    
    typedef typename Constraint<Q>::VectorQ VectorQ;
    typedef typename JacobianConstraint<Q,6>::Error Error;
    typedef typename JacobianConstraint<Q,6>::Jacobian Jacobian;
    
    ManipConstraint() { _initializeDefaults(); }
    ManipConstraint(int cspace_size) :
        RobotConstraintBase(), RobotJacobianConstraint<Q,6>(cspace_size)
    { _initializeDefaults(); }
    ManipConstraint(Manipulator& manipulator, const std::vector<size_t>& joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator)
    { _initializeDefaults(); }
    ManipConstraint(int cspace_size, Manipulator& manipulator, const std::vector<size_t>& joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator),
        RobotJacobianConstraint<Q,6>(cspace_size)
    { _initializeDefaults(); }
    
    virtual const Jacobian& getJacobian(const VectorQ& config, bool update=true) {
        if(!checkSetup()) { this->_Jacobian.setZero(); return this->_Jacobian; }
        
        if(update) _update(config);
        
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
    
    virtual const Error& getError(const VectorQ& config, bool fromCenter=false, bool update=true) {
        if(!checkSetup()) { this->_error.setZero(); return this->_error; }

        if(update) _update(config);

        Transform tf_error = _manip->withRespectTo(target.refFrame())
                             *target.respectToRef().inverse();
        const Eigen::Matrix3d& rot = tf_error.rotation().matrix();

        this->_displacement.template block<3,1>(0,0) =
                                        _manip->withRespectTo(target.refFrame()).translation()
                                            - target.respectToRef().translation();
        this->_displacement[3] =  atan2(rot(2,1), rot(2,2));
        this->_displacement[4] = -asin(rot(2,0));
        this->_displacement[5] =  atan2(rot(1,0), rot(0,0));
        for(int i=0; i<6; ++i)
        {
            if( this->_displacement[i] < min_limits[i] )
            {
                if(fromCenter && !std::isinf(max_limits[i]))
                    this->_error[i] = this->_displacement[i] - (min_limits[i]+max_limits[i])/2.0;
                else
                    this->_error[i] = this->_displacement[i] - min_limits[i];
            }
            else if( max_limits[i] < this->_displacement[i] )
            {
                if(fromCenter && !std::isinf(min_limits[i]))
                    this->_error[i] = this->_displacement[i] - (min_limits[i]+max_limits[i])/2.0;
                else
                    this->_error[i] = this->_displacement[i] - max_limits[i];
            }
            else
                this->_error[i] = 0;
        }
        
        if(this->error_weights.size() == 6)
            for(int i=0; i<6; ++i)
                this->_error[i] *= this->error_weights[i];
        
        return this->_error;
    }
    
protected:
    
    Error _displacement;

    void _initializeDefaults() {
         min_limits.setOnes(); max_limits.setOnes();
         min_limits *= -0.001;  max_limits *= 0.001;
         this->error_clamp = 0.2; this->dq_clamp = 0.2;
         this->computeErrorFromCenter = true;
         error_weights.resize(6); error_weights.setOnes();
         error_weights[3] = error_weights[4] = error_weights[5] = 0.1;
         _reconfigure();
    }
    
};

typedef ManipConstraint<Eigen::Dynamic> ManipConstraintX;

class CenterOfMassConstraintBase : public virtual RobotConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CenterOfMassConstraintBase();

    bool useRobotSupportPolygon;
    std::vector<Eigen::Vector2d> supportConvexHull;
    double min_height;
    double max_height;

};

template<int Q>
class CenterOfMassConstraint : public RobotJacobianConstraint<Q,3>,
        public CenterOfMassConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef typename Constraint<Q>::VectorQ VectorQ;
    typedef typename JacobianConstraint<Q,3>::Error Error;
    typedef typename JacobianConstraint<Q,3>::Jacobian Jacobian;

    CenterOfMassConstraint() { _initializeDefaults(); }
    CenterOfMassConstraint(int cspace_size) :
        RobotConstraintBase(), RobotJacobianConstraint<Q,3>(cspace_size)
    { _initializeDefaults(); }
    CenterOfMassConstraint(Robot& robot, const std::vector<size_t>& joints) :
        RobotConstraintBase(robot, joints)
    { _initializeDefaults(); }
    CenterOfMassConstraint(int cspace_size, Robot& robot, const std::vector<size_t>& joints) :
        RobotConstraintBase(robot, joints), RobotJacobianConstraint<Q,3>(cspace_size)
    { _initializeDefaults(); }

    virtual const Jacobian& getJacobian(const VectorQ& config, bool update=true) {
        if(!checkSetup()) { this->_Jacobian.setZero(); return this->_Jacobian; }
        
        if(update) _update(config);
        
        double total_mass = _robot->mass();
        
        for(size_t i=0; i<this->_joints.size(); ++i)
        {
            _ex.reset(this->_robot->const_joint(_joints[i]).const_childLink(), 
                      Robot::Explorer::DOWNSTREAM);
            
            Translation p;
            double mass = 0;
            double mass_l;
            const Link* nextLink;
            while( (nextLink = _ex.nextLink()) )
            {
                mass_l = nextLink->mass;
                p += mass_l*nextLink->com.respectToWorld();
                mass += mass_l;
                
                for(size_t m=0; m<nextLink->numManips(); ++m)
                {
                    const Manipulator& manip = nextLink->const_manip(m);
                    mass_l = manip.mass();
                    p += mass_l*manip.com().respectToWorld();
                    mass += mass_l;
                }
            }
            
            _point = p/mass;
            
            this->_Jacobian.template block<3,1>(0,i) = mass/total_mass*
                    this->_robot->const_joint(this->_joints[i]).Jacobian_posOnly(
                        _point, Frame::World(), false);
        }
        
        return this->_Jacobian;
    }
    
    virtual const Error& getError(const VectorQ &config, bool fromCenter=false, bool update=true) {
        if(!checkSetup()) { this->_error.setZero(); return this->_error; }
        
        if(update) _update(config);
        
        Translation com = _robot->com().respectToWorld();
        
        if(isInsideConvexHull(com.template block<2,1>(0,0), _robot->getSupportPolygon()))
        {
            this->_error.template block<2,1>(0,0).setZero();
        }
        else
        {
            if(fromCenter)
            {
                this->_error.template block<2,1>(0,0) = com.template block<2,1>(0,0) 
                                                            - this->_robot->getSupportCenter();
            }
            else
            {
                this->_error.template block<2,1>(0,0) = com.template block<2,1>(0,0)
                        - closestPointOnHull(com.template block<2,1>(0,0), 
                                             this->_robot->getSupportPolygon());
            }
        }
        
        if(fromCenter)
        {
            if( com[2] < this->min_height || this->max_height < com[2] )
                this->_error[2] = com[2] - 0.5*(this->min_height+this->max_height);
            else
                this->_error[2] = 0;
        }
        else
        {
            if( com[2] < this->min_height )
                this->_error[2] = com[2] - this->min_height;
            else if( this->max_height < com[2] )
                this->_error[2] = com[2] - this->max_height;
            else
                this->_error[2] = 0;
        }
        
        return this->_error;
    }

protected:

    Robot::Explorer _ex;
    KinTranslation _point;

    void _initializeDefaults() {
        this->error_clamp = 0.2; this->dq_clamp = 0.2;
        this->computeErrorFromCenter = true;
        error_weights.resize(3); error_weights.setOnes();
        _point.name("com_j");
        _reconfigure();
    }

};

typedef CenterOfMassConstraint<Eigen::Dynamic> CenterOfMassContraintX;

} // namespace akin

#endif // AKIN_ROBOTCONSTRAINT_H
