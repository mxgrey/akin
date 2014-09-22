#ifndef AKIN_SCREW_H
#define AKIN_SCREW_H

#include "Translation.h"

namespace akin {

class Screw : public Eigen::Matrix<double,6,1>
{
public:

    typedef Eigen::Matrix<double,6,1> Base;
    
    inline Screw(double x=0, double y=0, double z=0,
                 double u=0, double v=0, double w=0)
    {
        (*this) << x, y, z, u, v, w;
    }
    
    inline Screw(const Vec3& linear, const Vec3& angular)
    {
        (*this) << linear, angular;
    }

    template<typename OtherDerived>
    Screw(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double,6,1>(other) { }

    template<typename OtherDerived>
    Screw & operator= (const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

};


// TODO: Make a KinScrew

} // namespace akin

#endif // AKIN_SCREW_H
