#ifndef AKIN_CONSTRAINT_H
#define AKIN_CONSTRAINT_H

#include "akin/ConstraintBase.h"

namespace akin {

template<int Q>
class Constraint : public virtual ConstraintBase
{
public:
    
    typedef Eigen::Matrix<double,Q,1> VectorQ;
    typedef Eigen::Matrix<double,Q,Q> MatrixQ;
    
    Constraint() {
        useNullspace = false;
    }
    
    Validity getGradientX(Eigen::VectorXd &gradient, const Eigen::VectorXd &configuration) {
        _tempGradient = VectorQ(gradient);
        Validity v = getGradient(_tempGradient, VectorQ(configuration));
        gradient = Eigen::VectorXd(_tempGradient);
        return v;
    }
    virtual Validity getGradient(VectorQ& gradient, const VectorQ& configuration) = 0;
    
    Validity getValidityX(const Eigen::VectorXd &configuration) {
        return getValidity(VectorQ(configuration));
    }
    virtual Validity getValidity(const VectorQ& configuration) = 0;
    
    double getErrorNormX(const Eigen::VectorXd &configuration, bool update=true) {
        return getErrorNorm(VectorQ(configuration), update);
    }
    virtual double getErrorNorm(const VectorQ& configuration, bool update=true) = 0;
    
    virtual bool getNullspace(MatrixQ& Jnull, const VectorQ& configuration, bool update=true) = 0;
    
    int getConfigurationSize() { return Q; }
    
    bool useNullspace;
    
protected:
    
    VectorQ _tempGradient;
    
};

template<int Q, int W>
class JacobianConstraint : public Constraint<Q>
{
public:

    typedef typename ConstraintBase::Validity Validity;    
    typedef typename Constraint<Q>::VectorQ VectorQ;
    typedef Eigen::Matrix<double,W,Q> Jacobian;
    typedef Eigen::Matrix<double,Q,W> PseudoInverse;
    typedef Eigen::Matrix<double,W,1> Error;
    typedef Eigen::Matrix<double,W,W> MatrixW;
    typedef typename Constraint<Q>::MatrixQ MatrixQ;

    double damp_factor;
    bool computeErrorFromCenter;
    
    JacobianConstraint() {
        damp_factor=0.05;
        computeErrorFromCenter=true;
        this->error_clamp=0.2;
        this->component_clamp = 0.2;
    }
    
    virtual const Jacobian& getJacobian(const VectorQ& config, bool update=true) = 0;
    
    virtual const Error& getError(const VectorQ& config, 
                                  bool fromCenter=false, bool update=true) = 0;
    
    static void computeDampedPseudoInverse(PseudoInverse& pseudoInverse, const Jacobian& J,
                                           double damp_factor=0.05)
    {
        pseudoInverse = J.transpose()*(J*J.transpose()+
                            damp_factor*damp_factor*MatrixW::Identity()).inverse();
    }
    
    static void clampErrorNorm(Error& error, double clamp=0.2) {
        double norm = error.norm();
        if(norm > clamp)
            error *= clamp/norm;
    }
    
    static void clampGradientComponents(VectorQ& gradient, double clamp=0.2) {
        for(int i=0; i<Q; ++i)
            if(fabs(gradient[i]) > clamp)
                gradient[i] = gradient[i] > 0? clamp : -clamp;
    }
    
    virtual Validity getGradient(VectorQ &gradient, const VectorQ &configuration) {
        _update(configuration);
        
        getError(configuration, computeErrorFromCenter, false);
        getJacobian(configuration, false);
        computeDampedPseudoInverse(_pseudoInverse, _Jacobian, damp_factor);
        
        clampErrorNorm(_error);
        gradient = _pseudoInverse*_error;
        
        if(this->gradient_weights.size() == Q)
            for(int i=0; i<Q; ++i)
                gradient[i] *= this->gradient_weights[i];
        
        clampGradientComponents(gradient, this->component_clamp);
        
        return _computeCurrentValidity();
    }
    
    Validity getValidity(const VectorQ &configuration) {
        _update(configuration);
        
        getError(configuration, computeErrorFromCenter);
        return _computeCurrentValidity();
    }
    
    double getErrorNorm(const VectorQ &configuration, bool update) {
        if(update)
            getError(configuration, computeErrorFromCenter, true);
        
        return _error.norm();
    }
    
    bool getNullspace(MatrixQ &Jnull, const VectorQ& configuration, bool update) {
        if(!this->useNullspace)
            return false;
        
        if(update)
            getJacobian(configuration, true);
        
        computeDampedPseudoInverse(_pseudoInverse, _Jacobian, damp_factor);
        Jnull = MatrixQ::Identity() - _pseudoInverse*_Jacobian;
        return true;
    }
    
protected:
    
    virtual void _update(const VectorQ& config) = 0;
    
    virtual Validity _computeCurrentValidity() {
        for(int i=0; i<_error.size(); ++i)
            if(_error[i] != _error[i])
                return Validity::Invalid();

        if(_error.norm() == 0)
            return Validity::Valid();
        
        return Validity::Invalid();
    }
    
    Jacobian _Jacobian;
    PseudoInverse _pseudoInverse;
    Error _error;
    
};

} // namespace akin

#endif // AKIN_CONSTRAINT_H
