#ifndef AKIN_ANALYTICALIKBASE_H
#define AKIN_ANALYTICALIKBASE_H

#include "RobotConstraint.h"
#include "Solver.h"

namespace akin {

class AnalyticalIKBase : public ManipConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AnalyticalIKBase();
    
    AnalyticalIKBase();
    AnalyticalIKBase(Manipulator& manipulator);
    
    virtual void getSolutions(std::vector<Eigen::VectorXd>& solutions, 
                              bool exclude_invalid_solutions=true)=0;
    virtual Validity getBestSolution(Eigen::VectorXd& solution) = 0;
    virtual void selectBestSolution(Eigen::VectorXd& best, 
                                    std::vector<Eigen::VectorXd>& solutions) = 0;
    
    int options;
    
};

class NullAnalyticalIK : public AnalyticalIKBase, public NullConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    void getSolutions(std::vector<Eigen::VectorXd>& solutions, bool );
    Validity getBestSolution(Eigen::VectorXd& );
    virtual void selectBestSolution(Eigen::VectorXd& ,
                                    std::vector<Eigen::VectorXd>& );
    
};

} // namespace akin


#endif // AKIN_ANALYTICALIKBASE_H
