/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Jan 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Humanoid Robotics Lab nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AKIN_TRANSFORM_H
#define AKIN_TRANSFORM_H

#include "KinObject.h"
#include "Translation.h"
#include "Screw.h"
#include "Rotation.h"

namespace akin {


/*!
  * \class Transform
  * \brief Raw representation of a transform, derived from the Eigen C++ Isometry3d
  */

class Transform : public Eigen::Isometry3d
{
public:

    typedef Eigen::Isometry3d Base;

    /*!
      * \fn Transform()
      * \brief Every akin::Transform is initialized to identity unless the copy constructor is used
      */
    inline Transform() :
        Eigen::Isometry3d(Eigen::Isometry3d::Identity())
    {

    }
    
    inline Transform(const Translation& translation, const Rotation& rotation) :
        Eigen::Isometry3d(Eigen::Isometry3d::Identity())
    {
        translate(translation);
        rotate(rotation);
    }

    inline Transform(const Screw& screw) :
        Eigen::Isometry3d(Eigen::Isometry3d::Identity())
    {
        translate(screw.block<3,1>(0,0));
        rotate(Rotation(FreeVector(screw.block<3,1>(3,0))));
    }

    inline Transform(const Eigen::Isometry3d& tf) :
        Eigen::Isometry3d(tf)
    {

    }

    inline Screw diff(const akin::Transform& other) const
    {
        Screw result;

        result.block<3,1>(0,0) = this->translation() - other.translation();
//        const Eigen::Matrix3d& rot =
//                Eigen::AngleAxisd(this->rotation() * other.rotation().transpose()).matrix();

//        result[3] =  atan2(rot(2,1), rot(2,2));
//        result[4] = -asin(rot(2,0));
//        result[5] =  atan2(rot(1,0), rot(0,0));
        result.block<3,1>(3,0) = Rotation(this->rotation() * other.rotation().transpose())
                .getEulerAngles();

        return result;
    }

    inline Eigen::Vector3d getEulerAngles() const
    {
        return Rotation(rotation()).getEulerAngles();
    }

    inline akin::Transform operator*(const akin::Transform& other) const
    {
        return akin::Transform((Eigen::Isometry3d&)(*this) * (Eigen::Isometry3d&)(other));
    }

    inline akin::Translation operator*(const akin::Translation& other) const
    {
        return akin::Translation((Eigen::Isometry3d&)(*this) * (Eigen::Vector3d&)(other));
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const Transform& respectToWorld() const;
    Transform withRespectTo(const Frame& someFrame) const;

protected:

    void _update() const;
    mutable Transform _respectToWorld;
    
private:

};


} // namespace akin


inline std::ostream& operator<<(std::ostream& oStrStream, const akin::KinTransform& mTransform)
{
    oStrStream << (akin::KinObject&)mTransform << " has relative matrix:\n" << (akin::Transform&)mTransform
               << "\nAnd global matrix:\n" << mTransform.respectToWorld() << std::endl;
    return oStrStream;
}

#endif // AKIN_TRANSFORM_H
