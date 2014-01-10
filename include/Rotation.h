#ifndef ROTATION_H
#define ROTATION_H

#include "KinObject.h"

namespace akin {

/*!
 * \class Rotation
 * \brief Raw representation of a rotation, derived from the Eigen C++ Quaterniond
 *
 * Rotations are internally handled as quaternions, but can be manipulated as if they
 * are either quaternions by letting you access .w(), .x(), .y(), and .z() or as if
 * they are rotation matrices by letting you perform matrix arithmetic with them.
 */

class Rotation : public Eigen::Quaterniond
{
public:

    /*!
     * \fn Rotation(double angle, Axis rotation_axis)
     * \brief Angle-Axis style constructor
     * \param angle
     * \param rotation_axis
     */
    inline Rotation(double angle, const Axis& rotation_axis) :
        Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_axis))
    {

    }

    inline Rotation(const Eigen::AngleAxisd& someRotation) :
        Eigen::Quaterniond(someRotation)
    {

    }

    inline Rotation(const Eigen::Quaterniond& someRotation) :
        Eigen::Quaterniond(someRotation)
    {

    }

    inline Rotation(const Eigen::RotationBase<Eigen::Quaternion<double>,3>::RotationMatrixType someMatrix) :
        Eigen::Quaterniond(someMatrix)
    {

    }
};

} // namespace akin

#endif // ROTATION_H
