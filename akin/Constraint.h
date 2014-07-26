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

template<int N>
class JacobianConstraint : public Constraint<N>
{
public:

    typedef typename akin::ConstraintBase::Validity Validity;    
    typedef typename Constraint<N>::VectorN VectorN; 
    typedef Eigen::Matrix<double,6,N> Jacobian;
    typedef Eigen::Matrix<double,N,6> PseudoInverse;
    typedef Eigen::Matrix<double,6,1> Error;

    double damp_factor;
    bool computeErrorFromCenter;
    
    JacobianConstraint() {
        damp_factor=0.05; this->error_clamp=0.2; computeErrorFromCenter=true;
    }
    
    virtual const Jacobian& getJacobian(const VectorN& config, bool update=true) const = 0;
    
    static void computeDampedPseudoInverse(PseudoInverse& pseudoInverse, const Jacobian& J,
                                           double damp_factor=0.05)
    {
        pseudoInverse = J.transpose()*(J*J.transpose()+
                                       damp_factor*damp_factor*PseudoInverse::Identity()).inverse();
        return pseudoInverse;
    }
    
    static void clampErrorNorm(Error& error, double clamp=0.2) {
        double norm = error.norm();
        if(norm > clamp)
            error *= clamp/norm;
    }
    
    static void clampGradientNorm(VectorN& gradient, double clamp=0.2) {
        double norm = gradient.norm();
        if(norm > clamp)
            gradient *= clamp/norm;
    }
    
    virtual const Error& getError(const VectorN& config, 
                                  bool fromCenter=false, bool update=true) const = 0;
    
    virtual Validity getGradient(VectorN &gradient, const VectorN &configuration) {
        _update(configuration);
        
        getError(configuration, computeErrorFromCenter, false);
        getJacobian(configuration, false);
        computeDampedPseudoInverse(_pseudoInverse, _Jacobian, damp_factor);
        
        clampErrorNorm(_error);
        gradient = _pseudoInverse*_error;
        clampGradientNorm(gradient);
        
        return _computeCurrentValidity();
    }
    
protected:
    
    virtual void _update(const VectorN& config) = 0;
    
    virtual Validity _computeCurrentValidity() = 0;
    
    Jacobian _Jacobian;
    PseudoInverse _pseudoInverse;
    Error _error;
    
};

} // namespace akin

#endif // AKIN_CONSTRAINT_H
