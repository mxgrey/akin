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

#ifndef AKIN_ROTATION_H
#define AKIN_ROTATION_H

#include "KinObject.h"
#include "Translation.h"

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

    typedef Eigen::Quaterniond Base;

    /*!
     * \fn Rotation(double angle, Axis rotation_axis)
     * \brief Angle-Axis style constructor
     * \param angle
     * \param rotation_axis
     */
    inline Rotation(double angle, const Axis& rotation_axis) :
        Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_axis))
    {
        if(!rotation_axis.valid())
            setIdentity();
    }
    
    inline Rotation(const FreeVector& angular_displacement)
    {
        double _norm = angular_displacement.norm();
        if(_norm == 0)
            setIdentity();
        else
            (Eigen::Quaterniond&)(*this) = Eigen::Quaterniond(
                                    Eigen::AngleAxisd(_norm, angular_displacement/_norm));
    }

    inline Rotation(const Eigen::AngleAxisd& someRotation) :
        Eigen::Quaterniond(someRotation)
    {

    }

    inline Rotation(const Eigen::Quaterniond& someRotation) :
        Eigen::Quaterniond(someRotation)
    {

    }
    
    inline Rotation(double w=1, double x=0, double y=0, double z=0) :
        Eigen::Quaterniond(w, x, y, z)
    {
        
    }

    inline Eigen::Vector3d getEulerAngles() const
    {
        Eigen::Vector3d v;
        const Eigen::Matrix3d& rot = matrix();
        v[0] =  atan2(rot(2,1), rot(2,2));
        v[1] = -asin(rot(2,0));
        v[2] =  atan2(rot(1,0), rot(0,0));

        return v;
    }

    // This constructor allows you to construct MyVectorType from Eigen expressions
    template<typename OtherDerived>
    Rotation(const Eigen::RotationBase<OtherDerived, 3>& other) : Eigen::Quaterniond(other) { }
    template<typename OtherDerived>
    Rotation(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Quaterniond(other){ }
    
    // This method allows you to assign Eigen expressions to MyVectorType
    template<typename OtherDerived>
    Rotation & operator= (const Eigen::RotationBase <OtherDerived, 3>& other)
    {
        this->Base::operator=(other);
        return *this;
    }
};

} // namespace akin

#endif // AKIN_ROTATION_H
