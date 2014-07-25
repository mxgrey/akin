#ifndef AKIN_ROBOTCONSTRAINT_H
#define AKIN_ROBOTCONSTRAINT_H

#include "akin/Constraint.h"
#include "Robot.h"

namespace akin {

template<int N>
class RobotConstraint : public Constraint<N>
{
public:
    
    RobotConstraint() :
        _robot(NULL) { }
    
    RobotConstraint(const Robot* robot, const std::vector<size_t> joints) :
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
class ManipConstraint : public RobotConstraint<N>
{
public:
    
    ManipConstraint() :
        RobotConstraint() { }
    ManipConstraint(const Robot* robot, const std::vector<size_t> joints) :
        RobotConstraint(robot, joints) { }
    
protected:
    
    bool _reconfigure() {
        
    }
};



} // namespace akin

#endif // AKIN_ROBOTCONSTRAINT_H
