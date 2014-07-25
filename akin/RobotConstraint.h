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
    
    ManipConstraint() :
        RobotConstraintBase(), manip(NULL) { }
    ManipConstraint(const Robot* robot, const std::vector<size_t> joints) :
        RobotConstraintBase(robot, joints), 
        manip(NULL),  target(Frame::World(), "manip_target") { }
    ManipConstraint(const Manipulator* manipulator, const Robot* robot, 
                    const std::vector<size_t> joints) :
        RobotConstraintBase(robot, joints),
        manip(manipulator), target(Frame::World(), manipulator->name()+"_target")
    { target.respectToRef(manip->respectToWorld()); }
    
    const Manipulator* manip;
    Frame target;
    
    const Jacobian& getJacobian() const {
        _Jacobian.setZero();
        for(size_t i=0; i<_joints.size(); ++i)
            if(_dependency[i])
                _Jacobian.block<6,1>(0,i) = _robot->joint(_joints[i]).Jacobian(
                                                manip->point(), target.refFrame(), false);
        return _Jacobian;
    }
    
protected:
    
    Jacobian _Jacobian;
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
        for(size_t i=0; i<N; ++i)
        {
            const Joint& j = _robot->joint(_joints[i]);
            if(manip->descendsFrom(j.childLink()))
                _dependency.push_back(true);
            else
                _dependency.push_back(false);
        }
    }
};



} // namespace akin

#endif // AKIN_ROBOTCONSTRAINT_H
