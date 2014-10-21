#ifndef AKIN_SCREW_H
#define AKIN_SCREW_H

#include "Translation.h"

// TODO: Consider renaming this file to spatial
// and also using the convention of spatial coordinates

namespace akin {

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,6,6> Matrix6d;

class Screw : public Vector6d
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

    inline Eigen::Block<Vector6d, 3, 1, false> upper() { return block<3,1>(0,0); }
    inline const Eigen::Block<const Vector6d,3,1,false> upper() const { return block<3,1>(0,0); }

    inline Eigen::Block<Vector6d, 3, 1, false> linear() { return upper(); }
    inline const Eigen::Block<const Vector6d,3,1,false> linear() const { return upper(); }

    inline Eigen::Block<Vector6d, 3, 1, false> lower() { return block<3,1>(3,0); }
    inline const Eigen::Block<const Vector6d,3,1,false> lower() const { return block<3,1>(3,0); }

    inline Eigen::Block<Vector6d, 3, 1, false> angular() { return lower(); }
    inline const Eigen::Block<const Vector6d,3,1,false> angular() const { return lower(); }

};

class Spatial : public Vector6d
{
public:

    typedef Eigen::Matrix<double,6,1> Base;

    inline Spatial(double u=0, double v=0, double w=0,
                   double x=0, double y=0, double z=0)
    {
        (*this) << u, v, w, x, y, z;
    }

    inline Spatial(const Vec3& angular, const Vec3& linear)
    {
        (*this) << angular, linear;
    }

    template<typename OtherDerived>
    Spatial(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double,6,1>(other) { }

    template<typename OtherDerived>
    Spatial & operator= (const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    inline Eigen::Block<Vector6d, 3, 1, false> upper() { return block<3,1>(0,0); }
    inline const Eigen::Block<const Vector6d,3,1,false> upper() const { return block<3,1>(0,0); }

    inline Eigen::Block<Vector6d, 3, 1, false> lower() { return block<3,1>(3,0); }
    inline const Eigen::Block<const Vector6d,3,1,false> lower() const { return block<3,1>(3,0); }

};

// TODO: Make a KinScrew

} // namespace akin

#endif // AKIN_SCREW_H
