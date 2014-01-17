#ifndef TRANSLATION_H
#define TRANSLATION_H

#include "KinObject.h"

namespace akin {

/*!
 * \class Translation
 * \brief Raw representation of a translation, derived from the Eigen C++ Vector3d
 *
 * Translations are inherently three-dimensional, although excluding the z component will
 * default it to zero, which can be used to simulate two dimensions.
 *
 * The length of a translation should be expected to change if a Transform is applied which
 * has any non-zero translation components.
 */

class Translation : public Eigen::Vector3d
{
public:

    inline Translation(double x=0, double y=0, double z=0) :
        Eigen::Vector3d(x, y, z)
    {

    }

    inline Translation(const Eigen::Vector3d& vec) :
        Eigen::Vector3d(vec)
    {

    }
};

/*!
 * \class KinTranslation
 * \brief A Translation which keeps track of its kinematic relationships
 *
 * As a KinObject, any time the kinematic tree upstream of this transform is
 * changed, it will update its value with respect to the world.
 */
class KinTranslation : public Translation, public KinObject
{
public:
    
    KinMacro( KinTranslation, Translation )
//    KinTranslation(Frame& referenceFrame,
//                   std::string translationName="translation",
//                   verbosity::verbosity_level_t report_level = verbosity::INHERIT);

//    KinTranslation(const Translation& relativeTranslation,
//                   Frame& referenceFrame,
//                   std::string translationName="translation",
//                   verbosity::verbosity_level_t report_level = verbosity::INHERIT);

    const Translation& respectToWorld();
    Translation withRespectTo(Frame& someFrame);

protected:

    void _update();
    Translation _respectToWorld;
};

/*!
 * \class Point
 * \brief A Translation behaves the same way a Point would be expected to, so they are typedefed
 *
 */
typedef Translation Point;
typedef KinTranslation KinPoint;

/*!
 * \class FreeVector
 * \brief Raw representation of a free vector, derived from the Translation class
 *
 * FreeVector shares all the same operations as Translation. The key difference is
 * that when a homogeneous transformation is applied to a FreeVector, only the
 * rotation component is applied.
 *
 * This is useful for representing velocities and differences between Points or
 * differences between Translations.
 */
class FreeVector : public Translation
{
public:

    inline FreeVector(double x=0, double y=0, double z=0)
    {
        (Eigen::Vector3d&)(*this) = Eigen::Vector3d(x, y, z);
    }

    inline FreeVector(const Eigen::Vector3d& vec)
    {
        (Eigen::Vector3d&)(*this) = vec;
    }

};

/*!
 * \class KinFreeVector
 * \brief A FreeVector which keeps track of its kinematic relationships
 *
 * As a KinObject, any time the kinematic tree upstream of this transform is
 * changed, it will update its value with respect to the world.
 */
class KinFreeVector : public FreeVector, public KinObject
{
public:

    KinMacro( KinFreeVector, FreeVector )
//    KinFreeVector(Frame& referenceFrame,
//                  std::string freeVectorName="free_vector",
//                  verbosity::verbosity_level_t report_level = verbosity::INHERIT);

//    KinFreeVector(const FreeVector& relativeFreeVector,
//                  Frame& referenceFrame,
//                  std::string freeVectorName="free_vector",
//                  verbosity::verbosity_level_t report_level = verbosity::INHERIT);

    const FreeVector& respectToWorld();
    FreeVector withRespectTo(Frame& someFrame);

protected:

    void _update();
    FreeVector _respectToWorld;
};

/*!
 * \class Velocity
 * \brief A FreeVector behaves the same way a Velocity would be expected to, so they are typedefed
 */
typedef FreeVector Velocity;

/*!
 * \class Axis
 * \brief A raw representation of an axis, derived from FreeVector
 *
 * The only difference between an axis and a FreeVector is that an axis always ensures that it is
 * normalized.
 */
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

class KinAxis : public Axis, public KinObject
{
public:

    KinMacro( KinAxis, Axis )
//    KinAxis(Frame& referenceFrame,
//            std::string axisName="axis",
//            verbosity::verbosity_level_t report_level = verbosity::INHERIT);

//    KinAxis(const Axis& relativeAxis,
//            Frame& referenceFrame,
//            std::string axisName="axis",
//            verbosity::verbosity_level_t report_level = verbosity::INHERIT);

    const Axis& respectToWorld();
    Axis withRespectTo(Frame& someFrame);

protected:

    void _update();
    Axis _respectToWorld;

};

} // namespace akin

inline std::ostream& operator<<(std::ostream& oStrStream, akin::KinTranslation& mTranslation)
{
    oStrStream << (akin::KinObject&)mTranslation << " has relative translation:\n"
               << "<" << mTranslation.transpose() << ">\n"
               << "And global translation:\n"
               << "<" << mTranslation.respectToWorld().transpose() << ">" << std::endl;
    return oStrStream;
}

#endif // TRANSLATION_H
