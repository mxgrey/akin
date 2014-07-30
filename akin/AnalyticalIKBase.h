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
    
};

class NullAnalyticalIK : public AnalyticalIKBase, public NullConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    void getSolutionsX(std::vector<Eigen::VectorXd>& solutions, std::vector<bool>& valid);
    Validity getBestSolutionX(Eigen::VectorXd& );
    
};


template<int Q>
class AnalyticalIKSupport : public ManipConstraint<Q>, public virtual AnalyticalIKBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AnalyticalIKSupport() { }

    typedef typename ManipConstraint<Q>::VectorQ VectorQ;

    AnalyticalIKSupport() { }
    AnalyticalIKSupport(int cspace_size) :
        RobotConstraintBase(), AnalyticalIKBase(), ManipConstraint<Q>(cspace_size) { }
    AnalyticalIKSupport(Manipulator& manipulator, const std::vector<size_t> joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator) { }
    AnalyticalIKSupport(int cspace_size, Manipulator& manipulator,
                        const std::vector<size_t>& joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator),
        ManipConstraint<Q>(cspace_size, manipulator, joints) { }


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
            solutions[i] = Eigen::VectorXd(_solutions);
    }

    virtual void getSolutions(std::vector<VectorQ>& solutions,
                              std::vector<bool>& valid) = 0;

    virtual size_t selectBestSolution(VectorQ& best, const VectorQ& lastConfig,
                                      const std::vector<VectorQ>& solutions,
                                      const std::vector<bool>& valid) const {
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

        return result;
    }

    // TODO: Decide on a variety of criteria for deciding what is the best solution
    virtual double rateConfigCost(const VectorQ& config, const VectorQ& lastConfig) = 0;

protected:

    VectorQ _tempBest;
    std::vector<VectorQ> _solutions;
    std::vector<bool> _valid;
    std::vector<size_t> _validChoices;

};

typedef AnalyticalIKSupport<Eigen::Dynamic> AnalyticalIKSupportX;

} // namespace akin


#endif // AKIN_ANALYTICALIKBASE_H
