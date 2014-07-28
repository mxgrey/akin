
#include "akin/Solver.h"
#include <iostream>

using namespace akin;
using namespace std;

SolverX::SolverX() :
    step_size(1.0),
    max_steps(300),
    max_attempts(1),
    max_perturbation(3.0),
    perturb_seed(true),
    random_resolution(10000),
    _mandatory(NULL),
    _optimal(NULL)
{
    srand(time(NULL));
}

bool SolverX::setMandatoryConstraint(ConstraintBase *constraint)
{
    _mandatory = constraint;
    return true;
}

ConstraintBase* SolverX::getMandatoryConstraint()
{
    return _mandatory;
}

bool SolverX::setOptimalConstraint(ConstraintBase *constraint)
{
    _optimal = constraint;
    return true;
}

ConstraintBase* SolverX::getOptimalConstraint()
{
    return _optimal;
}

bool SolverX::checkLimits(const Eigen::VectorXd &config) const
{
    if(config.size() != min_limits.size() || config.size() != max_limits.size())
        return false;
    
    for(int i=0; i<config.size(); ++i)
        if( config[i] < min_limits[i] || max_limits[i] < config[i] )
            return false;
    
    return true;
}

void SolverX::_enforceLimits(Eigen::VectorXd &config) const
{
    for(int i=0; i<config.size(); ++i)
    {
        if(config[i] < min_limits[i])
            config[i] = min_limits[i];
        else if(max_limits[i] < config[i])
            config[i] = max_limits[i];
    }
}

bool SolverX::checkForNan(const Eigen::VectorXd &config) const
{
    for(int i=0; i<config.size(); ++i)
    {
        if( config[i] != config[i] )
        {
            std::cout << "NaN found in component #" << i << "!" << std::endl;
            return false;
        }
    }
    return true;
}

bool SolverX::checkValidity(const Eigen::VectorXd &config) const
{
    if(_mandatory == NULL)
        return true;
    
    return _mandatory->getValidityX(config).valid;
}

bool SolverX::randomizeConfig(Eigen::VectorXd &config) const
{
    if(config.size() != min_limits.size() || config.size() != max_limits.size())
        return false;
    
    double original = 0;
    for(int i=0; i<config.size(); ++i)
    {
        original = config[i];
        if(std::isinf(max_limits[i]) || std::isinf(min_limits[i]))
            config[i] = (2*((double)(rand()%random_resolution)/(double)(random_resolution-1))-1)
                        * max_perturbation;
        else
            config[i] = ((double)(rand()%random_resolution)/(double)(random_resolution-1))
                        * (max_limits[i] - min_limits[i]) + min_limits[i];
        
        if(fabs(config[i]-original) > max_perturbation)
            config[i] = config[i]-original > 0? 
                            original+max_perturbation : original-max_perturbation;
    }
    
    return true;
}

bool SolverX::solve(Eigen::VectorXd &config) const
{
    return _satisfy(config, _mandatory, max_attempts);
}

bool SolverX::optimize(Eigen::VectorXd &config) const
{
    return _satisfy(config, _optimal, 1);
}

bool SolverX::_satisfy(Eigen::VectorXd& config, ConstraintBase* constraint, size_t max_attempts_) const
{
    if(config.size() != min_limits.size() || config.size() != max_limits.size())
    {
        std::cout << "Configuration-Space sizes do not match:\n" 
                  << " -- Input:" << config.size() 
                  << " | Min:" << min_limits.size() 
                  << " | Max:" << max_limits.size() << std::endl;
        return false;
    }
    
    if(constraint == NULL)
        return true;
    
    const Eigen::VectorXd original_config = config;
    Eigen::VectorXd rConfig(config.size());
    ConstraintBase::Validity v = ConstraintBase::Validity::Invalid();
    Eigen::VectorXd gradient(config.size());
    size_t attempt_count = 1;
    
    do {
        
        size_t step_count = 1;
        do {
            
            v = constraint->getGradientX(gradient, config);
            if(!v.valid)
            {
                config = config - step_size*gradient;
            }
            
            if(!checkForNan(config))
                break;
            
            _enforceLimits(config);
            
            if(v.stuck)
                break;
            
            ++step_count;
            if(max_steps > 0 && step_count > max_steps)
                break;
            
        } while( !v.valid && !v.stuck );
        
        ++attempt_count;
        if(max_attempts_ > 0 && attempt_count > max_attempts_)
            break;
        
        if(!v.valid)
        {
            randomizeConfig(rConfig);
            double stochasticFactor = ((double)(rand()%random_resolution))
                                      /(double)(random_resolution-1);
            if(perturb_seed)
                config = original_config + (rConfig-original_config)*stochasticFactor;
            else
                config = config + (rConfig-config)*stochasticFactor;
        }
        
    } while( !v.valid );
    
    return checkValidity(config);
}




RobotSolverX::RobotSolverX(Robot &robot) :
    _robot(&robot)
{
    
}

bool RobotSolverX::setMandatoryConstraint(RobotConstraintBase *constraint)
{
    if(constraint==NULL)
    {
        _mandatory = NULL;
        return true;
    }
    
    if(constraint->getRobot() != _robot)
        return false;
    
    _mandatory = constraint;
    _joints = constraint->getJoints();
    resetLimits();
    return true;
}

bool RobotSolverX::setOptimalConstraint(RobotConstraintBase *constraint)
{
    if(constraint==NULL)
    {
        _optimal = NULL;
        return true;
    }
    
    if(constraint->getRobot() != _robot)
        return false;
    
    _optimal = constraint;
    _joints = constraint->getJoints();
    resetLimits();
    return true;
}

void RobotSolverX::resetLimits()
{
    min_limits.resize(_joints.size());
    max_limits.resize(_joints.size());
    
    for(size_t i=0; i<_joints.size(); ++i)
    {
        min_limits[i] = _robot->joint(_joints[i]).min();
        max_limits[i] = _robot->joint(_joints[i]).max();
    }
}

Robot* RobotSolverX::getRobot()
{
    return _robot;
}
