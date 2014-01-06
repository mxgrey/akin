#ifndef TRANSLATION_H
#define TRANSLATION_H

#include "KinObject.h"

namespace akin {

/*!
 * \class Translation
 * \brief Raw representation of a transform, derived from the Eigen C++ Vector3d
 */

class Translation : public Eigen::Vector3d
{
public:

    inline Translation(double x=0, double y=0, double z=0)
    {
        (Eigen::Vector3d&)(*this) = Eigen::Vector3d(x, y, z);
    }

    inline Translation(const Eigen::Vector3d& vec)
    {
        (Eigen::Vector3d&)(*this) = vec;
    }
};

class FreeVector : public Translation
{
public:


};

class Axis : public FreeVector
{
public:

    inline Axis(double x=0, double y=0, double z=0)
    {
        (Eigen::Vector3d&)(*this) = Eigen::Vector3d(x,y,z).normalized();
    }

    inline Axis(const Eigen::Vector3d& vec)
    {
        (Eigen::Vector3d&)(*this) = vec.normalized();
    }

};


} // namespace akin

#endif // TRANSLATION_H
