#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "KinObject.h"

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
    inline Transform()
    {
        (Eigen::Isometry3d&)(*this) =
                Eigen::Isometry3d::Identity();
    }
    
};


/*!
  * \class KinTransform
  * \brief A transform which keeps track of its kinematic relationships
  *
  * Like its inherited class Transform, you can use the template parameter to
  * set the precision to float (more efficient) or double (more precise).
  * The default is double.
  */

class KinTransform : public Transform, public KinObject
{
public:
    KinTransform(Frame& referenceFrame,
                 std::string tfName="transform",
                 verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
    KinTransform(const Transform& relativeTf,
                 Frame& referenceFrame,
                 std::string tfName="transform",
                 verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
};


} // namespace akin

inline std::ostream& operator<<(std::ostream& oStrStream,
                                const akin::Transform& mTransform)
{
    oStrStream << mTransform.matrix();
    return oStrStream;
}

std::ostream& operator<<(std::ostream& oStrStream, const akin::KinTransform& mTransform);

#endif // TRANSFORM_H
