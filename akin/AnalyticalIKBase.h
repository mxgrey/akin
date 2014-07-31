#ifndef AKIN_ANALYTICALIKBASE_H
#define AKIN_ANALYTICALIKBASE_H

#include "RobotConstraint.h"
#include "Solver.h"

namespace akin {

class AnalyticalIKBase : public virtual ManipConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AnalyticalIKBase();
    
    AnalyticalIKBase();
    
    virtual void getSolutionsX(std::vector<Eigen::VectorXd>& solutions,
                               std::vector<bool>& valid) = 0;
    virtual Validity getBestSolutionX(Eigen::VectorXd& solution) = 0;
    
    int options;
    bool ignore_joint_limits;
    
};

class NullAnalyticalIK : public AnalyticalIKBase, public NullConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    void getSolutionsX(std::vector<Eigen::VectorXd>& solutions, std::vector<bool>& valid);
    Validity getBestSolutionX(Eigen::VectorXd& );
    
};


template<int Q>
class AnalyticalIKTemplate : public ManipConstraint<Q>, public virtual AnalyticalIKBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AnalyticalIKTemplate() { }

    typedef typename ManipConstraint<Q>::VectorQ VectorQ;
    typedef typename ManipConstraint<Q>::Error Error;

    AnalyticalIKTemplate() : _goalTf(Frame::World(), "ik_goal") { _analyticalIKDefaults(); }
    AnalyticalIKTemplate(int cspace_size) :
        RobotConstraintBase(), AnalyticalIKBase(), ManipConstraint<Q>(cspace_size), 
        _goalTf(Frame::World(), "ik_goal") { _analyticalIKDefaults(); }
    AnalyticalIKTemplate(Manipulator& manipulator, const std::vector<size_t> joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator),
        _goalTf(Frame::World(), manipulator.name()+"_ik_goal") { _analyticalIKDefaults(); }
    AnalyticalIKTemplate(int cspace_size, Manipulator& manipulator,
                        const std::vector<size_t>& joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator),
        ManipConstraint<Q>(cspace_size, manipulator, joints),
        _goalTf(Frame::World(), manipulator.name()+"_ik_goal") { _analyticalIKDefaults(); }

    virtual Validity getGradient(VectorQ& gradient, const VectorQ& configuration) {
        _tempBest = configuration;
        Validity v = getBestSolution(_tempBest);
        gradient = configuration - _tempBest;
        return v;
    }

    virtual Validity getBestSolutionX(Eigen::VectorXd& best) {
        _tempBest = VectorQ(best);
        Validity v = getBestSolution(_tempBest);
        best = Eigen::VectorXd(_tempBest);
        return v;
    }

    virtual Validity getBestSolution(VectorQ& best) {
        getSolutions(_solutions, _valid);
        size_t selection = selectBestSolution(best, best, _solutions, _valid);
        if(_valid[selection])
            return Validity::Valid();

        return Validity::Stuck();
    }

    virtual void getSolutionsX(std::vector<Eigen::VectorXd>& solutions,
                               std::vector<bool>& valid) {
        getSolutions(_solutions, valid);
        solutions.resize(_solutions.size());
        for(size_t i=0; i<_solutions.size(); ++i)
            solutions[i] = Eigen::VectorXd(_solutions[i]);
    }

    virtual void getSolutions(std::vector<VectorQ>& solutions,
                              std::vector<bool>& valid) { }// = 0;

    virtual size_t selectBestSolution(VectorQ& best, const VectorQ& lastConfig,
                                      const std::vector<VectorQ>& solutions,
                                      const std::vector<bool>& valid) {
        _validChoices.clear();
        for(size_t i=0; i<valid.size(); ++i)
            if(valid[i])
                _validChoices.push_back(i);

        bool valid_exists = _validChoices.size() > 0;
        size_t result = (size_t)(-1); double best_cost = INFINITY;
        double cost=0;
        if(valid_exists)
        {
            for(size_t v=0; v<_validChoices.size(); ++v)
            {
                const size_t& i=_validChoices[v];
                cost = rateConfigCost(solutions[i], lastConfig);
                if( cost < best_cost )
                {
                    result = i;
                    best_cost = cost;
                }
            }
        }
        else
        {
            for(size_t i=0; i<solutions.size(); ++i)
            {
                cost = rateConfigCost(solutions[i], lastConfig);
                if( cost < best_cost )
                {
                    result = i;
                    best_cost = cost;
                }
            }
        }

        best = solutions[result];
        return result;
    }
    
    const KinTransform& getGoalTransform(const VectorQ& config) {
        getError(config, this->computeErrorFromCenter);
        _goalTf.changeRefFrame(this->target.refFrame());
//        _goalTf = _manip->withRespectTo(target.refFrame());
        std::cout << "Error: " << this->_error.transpose() << std::endl;
        
        Error _target_disp = this->_displacement - this->_error;
        std::cout << "disp:  " << _target_disp.transpose() << std::endl;

        Transform tf_target_disp;

        tf_target_disp.translate(_target_disp.template block<3,1>(0,0));
        tf_target_disp.rotate(Rotation(_target_disp[3], Vec3(1,0,0)));
        tf_target_disp.rotate(Rotation(_target_disp[4], Vec3(0,1,0)));
        tf_target_disp.rotate(Rotation(_target_disp[5], Vec3(0,0,1)));

//        _goalTf = /*this->target.respectToRef() **/ _goalTf * tf_goal_t0.inverse();
        _goalTf = tf_target_disp * target.respectToRef();


//        _goalTf.pretranslate(-this->_error.template block<3,1>(0,0));
        
        
//        _goalTf.prerotate(Rotation(-this->_error[5], Vec3(0,0,1)));
//        _goalTf.prerotate(Rotation(-this->_error[4], Vec3(0,1,0)));
//        _goalTf.prerotate(Rotation(-this->_error[3], Vec3(1,0,0)));
        
//        _goalTf.rotate(Rotation(-this->_error[3], Vec3(1,0,0)));
//        _goalTf.rotate(Rotation(-this->_error[4], Vec3(0,1,0)));
//        _goalTf.rotate(Rotation(-this->_error[5], Vec3(0,0,1)));
        return _goalTf;
    }

    // TODO: Decide on a variety of criteria for deciding what is the best solution
    virtual double rateConfigCost(const VectorQ& config, const VectorQ& lastConfig) {
        return (config-lastConfig).norm();
    }

protected:

    void _analyticalIKDefaults() {
        this->error_weights.resize(6); error_weights.setOnes();
    }

    VectorQ _tempBest;
    std::vector<VectorQ> _solutions;
    std::vector<bool> _valid;
    std::vector<size_t> _validChoices;
    KinTransform _goalTf;
    Rotation _errorRot;

};

typedef AnalyticalIKTemplate<Eigen::Dynamic> AnalyticalIKSupportX;

} // namespace akin


#endif // AKIN_ANALYTICALIKBASE_H
