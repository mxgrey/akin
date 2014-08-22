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

    Validity& operator&=(const Validity& otherV);
    Validity& operator|=(const Validity& otherV);

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
    
    ConstraintBase();
    
    virtual Validity getGradientX(Eigen::VectorXd& gradient, 
                                     const Eigen::VectorXd& configuration) = 0;

    virtual void setConfiguration(const Eigen::VectorXd& config) = 0;
    virtual Validity computeGradient() = 0;
    virtual double getGradientComponent(size_t i) const = 0;
    
    virtual Validity getValidityX(const Eigen::VectorXd& configuration) = 0;
    
    virtual double getErrorNormX(const Eigen::VectorXd& configuration, bool update=true) = 0;
    
    virtual int getConfigurationDimension() = 0;
    
    Eigen::VectorXd gradient_weights;
    Eigen::VectorXd error_weights;
    double error_clamp;
    double dq_clamp;
    double damp_factor;
    bool computeErrorFromCenter;

    bool isNull() const;

protected:

    bool _isNull;

};

class NullConstraintBase : public virtual ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    virtual ~NullConstraintBase();
    NullConstraintBase();

    virtual void setConfiguration(const Eigen::VectorXd&);
    virtual Validity computeGradient();
    virtual double getGradientComponent(size_t) const;
    
    virtual Validity getGradientX(Eigen::VectorXd &gradient, const Eigen::VectorXd &configuration);
    virtual Validity getValidityX(const Eigen::VectorXd &);
    virtual double getErrorNormX(const Eigen::VectorXd &, bool update=true);
    virtual int getConfigurationDimension();
    
protected:
    
};

}

// Definition in Constraint.cpp
std::ostream& operator<<(std::ostream& oStrStream, const akin::Validity& v);

#endif // CONSTRAINTBASE_H
