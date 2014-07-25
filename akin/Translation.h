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

#ifndef AKIN_TRANSLATION_H
#define AKIN_TRANSLATION_H

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
    
    inline Translation(const Eigen::Vector3d::ScalarMultipleReturnType& vec) :
        Eigen::Vector3d(vec)
    {
        
    }

    inline Translation(const Eigen::Vector3d& vec) :
        Eigen::Vector3d(vec)
    {

    }

    inline Translation(const Eigen::Block<const Eigen::Matrix<double, 4, 4>, 3, 1, false>& vec) :
        Eigen::Vector3d(vec)
    {

    }

    // Returned by Transform operations
    inline Translation(const Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<double>, Eigen::Matrix<double, 3, 1> >& vec) :
        Eigen::Vector3d(vec)
    {

    }
    
    // Returned by Rotation operations, but conflicts with the one below
//    inline Translation(const Eigen::CoeffBasedProduct<const Eigen::Matrix<double,3,3>&,
//                       const Eigen::Matrix<double, 3, 1>&, 6>& vec) :
//        Eigen::Vector3d(vec)
//    {
        
//    }
    
    // Returned when a negative is put in front of a rotation
    inline Translation(const Eigen::CoeffBasedProduct<
                       const Eigen::CwiseUnaryOp<Eigen::internal::scalar_opposite_op<double>,
                       const Eigen::Matrix<double,3,3> >,
                       const Eigen::Matrix<double,3,1>&, 6 >& vec) :
        Eigen::Vector3d(vec)
    {
        
    }
    
    
    inline Translation& operator=(const Eigen::Vector3d& vec)
    {
        (Eigen::Vector3d&)(*this) = vec;
        return *this;
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

    const Translation& respectToWorld() const;
    Translation withRespectTo(const Frame& someFrame) const;

protected:

    void _update() const;
    mutable Translation _respectToWorld;
};

/*!
 * \class Point
 * \brief A Translation behaves the same way a Point would be expected to, so they are typedefed
 *
 */
typedef Translation Point;
typedef Translation Vec3;
typedef KinTranslation KinPoint;
typedef KinTranslation KinVec3;

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

    const FreeVector& respectToWorld() const;
    FreeVector withRespectTo(const Frame& someFrame) const;

protected:

    void _update() const;
    mutable FreeVector _respectToWorld;
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

//    inline Axis(const Eigen::Vector3d& vec)
//    {
//        (Eigen::Vector3d&)(*this) = vec.normalized();
//    }
    
    inline Axis(const Translation& vec)
    {
        (Eigen::Vector3d&)(*this) = vec.normalized();
    }
    
//    inline Axis(const Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<double>, Eigen::Matrix<double, 3, 1> >& vec)
//    {
//        (Eigen::Vector3d&)(*this) = vec.normalized();
//    }
    
    inline Axis& operator=(const Eigen::Vector3d& vec)
    {
        (Eigen::Vector3d&)(*this) = vec.normalized();
        return *this;
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

    const Axis& respectToWorld() const;
    Axis withRespectTo(const Frame& someFrame) const;

protected:

    void _update() const;
    mutable Axis _respectToWorld;

};

} // namespace akin

inline std::ostream& operator<<(std::ostream& oStrStream, const akin::KinTranslation& mTranslation)
{
    oStrStream << (akin::KinObject&)mTranslation << " has relative translation:\n"
               << "<" << mTranslation.transpose() << ">\n"
               << "And global translation:\n"
               << "<" << mTranslation.respectToWorld().transpose() << ">" << std::endl;
    return oStrStream;
}

#endif // AKIN_TRANSLATION_H
