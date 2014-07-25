#ifndef AKIN_SCREW_H
#define AKIN_SCREW_H

#include "Translation.h"

namespace akin {

class Screw : public Eigen::Matrix<double,6,1>
{
public:
    
    inline Screw(double x=0, double y=0, double z=0,
                 double u=0, double v=0, double w=0)
    {
        (*this) << x, y, z, u, v, w;
    }
    
    inline Screw(const Vec3& linear, const Vec3& angular)
    {
        (*this) << linear, angular;
    }
    
    inline Screw(const Eigen::Matrix<double,6,1>& screw) :
        Eigen::Matrix<double,6,1>(screw)
    {
        
    }
    
    inline Screw(const Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<double>, Eigen::Matrix<double, 6, 1> >& vec) :
        Eigen::Matrix<double, 6, 1>(vec)
    {
        
    }
    
    // TODO: Handle multiplication by transforms and rotations
    
    inline Screw& operator=(const Eigen::Matrix<double,6,1>& screw)
    {
        (Eigen::Matrix<double,6,1>)(*this) = screw;
        return *this;
    }
    
};


// TODO: Make a KinScrew

} // namespace akin

#endif // AKIN_SCREW_H
