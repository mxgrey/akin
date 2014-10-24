#ifndef AKIN_SOLVER_H
#define AKIN_SOLVER_H

#include "akin/RobotConstraint.h"
#include <vector>
#include <map>

namespace akin {

class SolverX
{
public:
    
    SolverX();
    inline virtual ~SolverX() { }
    
    virtual bool setMandatoryConstraint(ConstraintBase* constraint, size_t config_size);
    ConstraintBase* getMandatoryConstraint();
    virtual bool setOptimalConstraint(ConstraintBase* constraint);
    ConstraintBase* getOptimalConstraint();
    bool checkLimits(const Eigen::VectorXd& config) const;
    bool checkForNan(const Eigen::VectorXd& config) const;
    bool checkValidity(const Eigen::VectorXd& config) const;
    bool randomizeConfig(Eigen::VectorXd& config) const;
    
    bool solve(Eigen::VectorXd& config) const;
    bool optimize(Eigen::VectorXd& config) const;

    size_t expectedConfigSize() const;
    
    const Eigen::VectorXd& min_limits() const;
    bool min_limits(const Eigen::VectorXd& newLimits);
    const Eigen::VectorXd& max_limits() const;
    bool max_limits(const Eigen::VectorXd& newLimits);

    double step_size;
    size_t max_steps;
    size_t max_attempts;
    double max_perturbation;
    bool perturb_seed;
    int random_resolution;
    
protected:
    
    void _enforceLimits(Eigen::VectorXd& config) const;
    bool _satisfy(Eigen::VectorXd& config, ConstraintBase* constraint, size_t max_attempts_) const;
    
    ConstraintBase* _mandatory;
    ConstraintBase* _optimal;
    
    Eigen::VectorXd _min_limits;
    Eigen::VectorXd _max_limits;
    size_t _expectedConfigSize;
};


class RobotSolverX : public SolverX
{
public:
    
    RobotSolverX(Robot& robot);
    inline virtual ~RobotSolverX() { }
    
    bool setMandatoryConstraint(RobotConstraintBase* constraint);
    bool setOptimalConstraint(RobotConstraintBase* constraint);
    
    void resetLimits();
    
    Robot* getRobot();
    
protected:
    
    std::vector<size_t> _dofs;
    Robot* _robot;
    
};


} // namespace akin


#endif // AKIN_SOLVER_H
