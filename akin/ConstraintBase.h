#ifndef AKIN_CONSTRAINTBASE_H
#define AKIN_CONSTRAINTBASE_H

#include <string>
#include <Eigen/Core>

namespace akin {

class ConstraintBase
{
public:
    
    class Validity
    {
    public:
        
        Validity();
        bool valid;
        bool stuck;
        bool near_edge;
        
        std::string toString() const;
        
    }; // Definitions are given in Constraint.cpp
    
    virtual Validity getGradientX(Eigen::VectorXd& gradient, 
                                     const Eigen::VectorXd& configuration) = 0;
    
    virtual Validity getValidityX(const Eigen::VectorXd& configuration) = 0;
    
    virtual double getErrorNormX(const Eigen::VectorXd& configuration) = 0;
    
    virtual int getConfigurationSize() = 0;
    
    Eigen::VectorXd gradient_weights;
    Eigen::VectorXd error_weights;
    double error_clamp;
    double component_clamp;
};

}

// Definition in Constraint.cpp
std::ostream& operator<<(std::ostream& oStrStream, const akin::ConstraintBase::Validity& v);

#endif // CONSTRAINTBASE_H
