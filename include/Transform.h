#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "KinObject.h"
#include "Translation.h"
#include "Rotation.h"

namespace akin {


/*!
  * \class Transform
  * \brief Raw representation of a transform, derived from the Eigen C++ Isometry3d
  */

class Transform : public Eigen::Isometry3d
{
public:
    /*!
      * \fn Transform()
      * \brief Every akin::Transform is initialized to identity unless the copy constructor is used
      */
    inline Transform() :
        Eigen::Isometry3d(Eigen::Isometry3d::Identity())
    {

    }
    
    inline Transform(const Translation& translation, const Rotation& rotation = Rotation::Identity()) :
        Eigen::Isometry3d(Eigen::Isometry3d::Identity())
    {
        translate(translation);
        rotate(rotation);
    }

    inline Transform(const Eigen::Isometry3d& tf) :
        Eigen::Isometry3d(tf)
    {

    }

    inline akin::Transform operator*(const akin::Transform& other) const
    {
        return akin::Transform((Eigen::Isometry3d&)(*this) * (Eigen::Isometry3d&)(other));
    }

    inline akin::Translation operator*(const akin::Translation& other) const
    {
        return akin::Translation((Eigen::Isometry3d&)(*this) * (Eigen::Vector3d&)(other));
    }

    inline Eigen::Vector3d operator*(const Eigen::Vector3d& other) const
    {
        return (Eigen::Isometry3d&)(*this) * other;
    }

    inline akin::FreeVector operator*(const akin::FreeVector& other) const
    {
        return FreeVector(rotation() * (Eigen::Vector3d&)(other));
    }

    inline akin::Rotation operator*(const akin::Rotation& other) const
    {
        return Rotation(rotation() * (Eigen::Quaterniond&)(other));
    }
    
};

} // namespace akin
inline std::ostream& operator<<(std::ostream& oStrStream,
                                const akin::Transform& mTransform)
{
    oStrStream << mTransform.matrix();
    return oStrStream;
}


namespace akin {

/*!
  * \class KinTransform
  * \brief A Transform which keeps track of its kinematic relationships
  *
  * As a KinObject, any time the kinematic tree upstream of this transform is
  * changed, it will update its value with respect to the world.
  */

class KinTransform : public Transform, public KinObject
{
public:
    KinMacro( KinTransform, Transform )

    const Transform& respectToWorld();
    
    Transform withRespectTo(Frame& someFrame);

protected:

    void _update();
    Transform _respectToWorld;
    
private:

};


} // namespace akin


inline std::ostream& operator<<(std::ostream& oStrStream, akin::KinTransform& mTransform)
{
    oStrStream << (akin::KinObject&)mTransform << " has relative matrix:\n" << (akin::Transform&)mTransform
               << "\nAnd global matrix:\n" << mTransform.respectToWorld() << std::endl;
    return oStrStream;
}

#endif // TRANSFORM_H
