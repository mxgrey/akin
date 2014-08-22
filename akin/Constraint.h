#ifndef AKIN_CONSTRAINT_H
#define AKIN_CONSTRAINT_H

#include "akin/ConstraintBase.h"
#include <iostream>

namespace akin {

template<int Q>
class Constraint : public virtual ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~Constraint() { }
    
    typedef Eigen::Matrix<double,Q,1> VectorQ;
    typedef Eigen::Matrix<double,Q,Q> MatrixQ;
    
    Constraint() {
        useNullspace = false;
        if(Q==-1)
        {
            std::cout << "You should not use the default constructor for Dynamically Sized Constraints!"
                      << "\n -- Use the Constraint constructor with an 'int' argument!" << std::endl;
        }
        _config_dim = Q;
    }
    Constraint(int cspace_size) {
        useNullspace = false;
        if(Q==-1 || Q==cspace_size)
        {
            _config_dim = cspace_size;
            _config.resize(_config_dim);
            _gradient.resize(_config_dim);
            _tempGradient.resize(_config_dim);
        }
        else
        {
            std::cout << "Mismatch between template argument and constructor argument for " 
                      << "Dynamically Sized Constraint: \n -- Template:" << Q 
                      << ", Constructor:" << cspace_size << std::endl;
            _config_dim = 0;
        }
    }
    
    Validity getGradientX(Eigen::VectorXd &gradient, const Eigen::VectorXd &configuration) {
        _tempGradient.resize(getConfigurationDimension());
        Validity v = getGradient(_tempGradient, VectorQ(configuration));
        gradient = Eigen::VectorXd(_tempGradient);
        return v;
    }
    virtual Validity getGradient(VectorQ& gradient, const VectorQ& configuration) = 0;
    
    virtual void setConfiguration(const Eigen::VectorXd& config) {
        this->_config = VectorQ(config);
    }

    virtual Validity computeGradient() {
        return getGradient(_gradient, _config);
    }
    
    virtual double getGradientComponent(size_t i) const { return _gradient[i]; }
    
    Validity getValidityX(const Eigen::VectorXd &configuration) {
        return getValidity(VectorQ(configuration));
    }
    virtual Validity getValidity(const VectorQ& configuration) = 0;
    
    double getErrorNormX(const Eigen::VectorXd &configuration, bool update=true) {
        return getErrorNorm(VectorQ(configuration), update);
    }
    virtual double getErrorNorm(const VectorQ& configuration, bool update=true) = 0;
    
    virtual bool getNullspace(MatrixQ& Jnull, const VectorQ& configuration, bool update=true) = 0;
    
    int getConfigurationDimension() { return _config_dim; }
    
    bool useNullspace;
    
protected:
    
    int _config_dim;
    VectorQ _tempGradient;
    VectorQ _gradient;
    VectorQ _config;
    
};

typedef Constraint<Eigen::Dynamic> ConstraintX;

class JacobianConstraintBase : public virtual ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~JacobianConstraintBase();

    virtual void computeJacobian() = 0;
    virtual double getJacobianComponent(size_t i, size_t j) const = 0;
    virtual bool computeJPinvJ() = 0;
    virtual double getJPinvJComponent(size_t i, size_t j) const = 0;

    virtual void computeError() = 0;
    virtual double getErrorComponent(size_t i) = 0;
    virtual int getErrorDimension() = 0;

};

class NullJacobianConstraint : public virtual JacobianConstraintBase, public virtual NullConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~NullJacobianConstraint();

    virtual void computeJacobian();
    virtual double getJacobianComponent(size_t, size_t) const;
    virtual bool computeJPinvJ();
    virtual double getJPinvJComponent(size_t, size_t) const;

    virtual void computeError();
    virtual double getErrorComponent(size_t);
    virtual int getErrorDimension();

};

template<int Q, int W>
class JacobianConstraint : public Constraint<Q>, public virtual JacobianConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~JacobianConstraint() { }

    typedef typename Constraint<Q>::VectorQ VectorQ;
    typedef Eigen::Matrix<double,W,Q> Jacobian;
    typedef Eigen::Matrix<double,Q,W> PseudoInverse;
    typedef Eigen::Matrix<double,W,1> Error;
    typedef Eigen::Matrix<double,W,W> MatrixW;
    typedef typename Constraint<Q>::MatrixQ MatrixQ;
    
    JacobianConstraint() {
        _initJacobianConstraint();
    }
    
    JacobianConstraint(int cspace_dim) : Constraint<Q>(cspace_dim) {
        _initJacobianConstraint();
    }

    JacobianConstraint(int cspace_dim, int error_dim) : Constraint<Q>(cspace_dim) {
        if(W==-1 || W==error_dim)
        {
            _error_dim = error_dim;
            _error.resize(_error_dim);
        }
        else
        {
            std::cout << "Mismatch between template argument and constructor argument for "
                      << "Dynamically Sized Constraint: \n -- Template: " << Q
                      << ", Constructor:" << error_dim << std::endl;
            _error_dim = 0;
        }
    }
    
    virtual const Jacobian& getJacobian(const VectorQ& config, bool update=true) = 0;
    
    virtual const Error& getError(const VectorQ& config, 
                                  bool fromCenter=false, bool update=true) = 0;
    
    static void computeDampedPseudoInverse(PseudoInverse& pseudoInverse, const Jacobian& J,
                                           double damp_factor=0.05)
    {
        pseudoInverse = J.transpose()*(J*J.transpose()+
                            damp_factor*damp_factor*MatrixW::Identity(J.rows(),J.rows())).inverse();
    }
    
    static void clampErrorNorm(Error& error, double clamp=0.2) {
        double norm = error.norm();
        if(norm > clamp)
            error *= clamp/norm;
    }
    
    static void clampGradientComponents(VectorQ& gradient, double clamp=0.2) {
        for(int i=0; i<gradient.size(); ++i)
            if(fabs(gradient[i]) > clamp)
                gradient[i] = gradient[i] > 0? clamp : -clamp;
    }
    
    virtual Validity getGradient(VectorQ &gradient, const VectorQ &configuration) {
        _update(configuration);
        
        getError(configuration, this->computeErrorFromCenter, false);
        if(this->_error.norm() == 0)
        {
            gradient.setZero();
            return Validity::Valid();
        }
        
        getJacobian(configuration, false);
        computeDampedPseudoInverse(_pseudoInverse, _Jacobian, this->damp_factor);
        
        clampErrorNorm(_error);
        gradient = _pseudoInverse*_error;
        
        if(this->gradient_weights.size() == this->_config_dim)
            for(int i=0; i<this->_config_dim; ++i)
                gradient[i] *= this->gradient_weights[i];
        
        clampGradientComponents(gradient, this->dq_clamp);
        
        return _computeCurrentValidity();
    }
    
    Validity getValidity(const VectorQ &configuration) {
        _update(configuration);
        
        getError(configuration, this->computeErrorFromCenter);
        return _computeCurrentValidity();
    }
    
    double getErrorNorm(const VectorQ &configuration, bool update) {
        if(update)
            getError(configuration, this->computeErrorFromCenter, true);
        
        return _error.norm();
    }
    
    bool getNullspace(MatrixQ &Jnull, const VectorQ& configuration, bool update) {
        if(!this->useNullspace)
            return false;
        
        if(update)
            getJacobian(configuration, true);
        
        computeDampedPseudoInverse(_pseudoInverse, _Jacobian, this->damp_factor);
        Jnull = MatrixQ::Identity(this->_config_dim,this->_config_dim) - _pseudoInverse*_Jacobian;
        return true;
    }

    virtual void computeJacobian() {
        this->getJacobian(this->_config, true);
    }

    virtual double getJacobianComponent(size_t i, size_t j) const { return this->_Jacobian(i,j); }

    virtual bool computeJPinvJ() {
        if(!this->useNullspace) return false;

        computeDampedPseudoInverse(this->_pseudoInverse, this->_Jacobian, this->damp_factor);
        _JPinvJ = this->_pseudoInverse*this->_Jacobian;
        return true;
    }

    virtual double getJPinvJComponent(size_t i, size_t j) const { return _JPinvJ(i,j); }

    virtual void computeError() {
        getError(this->_config, computeErrorFromCenter);
    }

    virtual double getErrorComponent(size_t i) {
        return this->_error[i];
    }

    virtual int getErrorDimension() { return _error_dim; }
    
protected:

    int _error_dim;
    
    virtual void _update(const VectorQ& config) = 0;
    
    virtual Validity _computeCurrentValidity() {
        if(_error.norm() == 0)
            return Validity::Valid();
        
        for(int i=0; i<_error.size(); ++i)
        {
            if(_error[i] != _error[i])
            {
                std::cout << "Error vector has NaN: " << _error.transpose() << std::endl;
                return Validity::Stuck();
            }
        }
        
        return Validity::Invalid();
    }
    
    void _initJacobianConstraint() {
        this->damp_factor=0.05;
        this->computeErrorFromCenter=true;
        this->error_clamp=0.2;
        this->dq_clamp = 0.2;
        _Jacobian.resize(Eigen::NoChange,this->_config_dim);
        _pseudoInverse.resize(this->_config_dim,Eigen::NoChange);
    }
    
    Jacobian _Jacobian;
    PseudoInverse _pseudoInverse;
    Error _error;

    MatrixQ _JPinvJ;
    
};

} // namespace akin

#endif // AKIN_CONSTRAINT_H
