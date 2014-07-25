#ifndef AKIN_CONSTRAINT_H
#define AKIN_CONSTRAINT_H

#include "akin/ConstraintBase.h"

namespace akin {

template<int N>
class Constraint : public ConstraintBase
{
public:
    
    typedef Eigen::Matrix<double,N,1> VectorN;
    typedef Eigen::Matrix<double,N,N> MatrixN;
    
    Validity getGradientX(Eigen::VectorXd &gradient, const Eigen::VectorXd &configuration)
    {
        _tempGradient = VectorN(gradient);
        Validity v = getGradient(_tempGradient, VectorN(configuration));
        gradient = Eigen::VectorXd(_tempGradient);
        return v;
    }
    virtual Validity getGradient(VectorN& gradient, const VectorN& configuration) = 0;
    
    Validity getValidityX(const Eigen::VectorXd &configuration)
    {
        return getValidity(VectorN(configuration));
    }
    virtual Validity getValidity(const VectorN& configuration) = 0;
    
    double getErrorNormX(const Eigen::VectorXd &configuration)
    {
        return getErrorNorm(VectorN(configuration));
    }
    virtual double getErrorNorm(const VectorN& configuration) = 0;
    
    virtual bool getNullspace(MatrixN& Jnull) = 0;
    
protected:
    
    VectorN _tempGradient;
    
};

} // namespace akin

#endif // AKIN_CONSTRAINT_H
