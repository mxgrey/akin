#ifndef AKIN_CONSTRAINTBASE_H
#define AKIN_CONSTRAINTBASE_H

#include <string>
#include <Eigen/Core>

namespace akin {

class Validity
{
public:

    static Validity Valid();
    static Validity Stuck();
    static Validity Invalid();

    friend bool operator==(const Validity& v1, const Validity& v2);

    Validity();
    bool valid;
    bool stuck;
    bool near_edge;

    std::string toString() const;

}; // Definitions are given in Constraint.cpp

class ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    virtual ~ConstraintBase();
    
    virtual Validity getGradientX(Eigen::VectorXd& gradient, 
                                     const Eigen::VectorXd& configuration) = 0;
    
    virtual Validity getValidityX(const Eigen::VectorXd& configuration) = 0;
    
    virtual double getErrorNormX(const Eigen::VectorXd& configuration, bool update=true) = 0;
    
    virtual int getConfigurationSize() = 0;
    
    Eigen::VectorXd gradient_weights;
    Eigen::VectorXd error_weights;
    double error_clamp;
    double dq_clamp;
    double damp_factor;
    bool computeErrorFromCenter;

};

class NullConstraintBase : public virtual ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    virtual ~NullConstraintBase();
    
    Validity getGradientX(Eigen::VectorXd &gradient, const Eigen::VectorXd &configuration);
    Validity getValidityX(const Eigen::VectorXd &);
    double getErrorNormX(const Eigen::VectorXd &, bool update=true);
    int getConfigurationSize();
    
protected:
    
};

}

// Definition in Constraint.cpp
std::ostream& operator<<(std::ostream& oStrStream, const akin::Validity& v);

#endif // CONSTRAINTBASE_H
