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

#include "akin/AkinIncludes.h"

using namespace akin;
using namespace std;

void test_operators()
{
    Transform tf1;
    tf1.translate(Eigen::Vector3d(10, 5, 1));

    Transform tf2;
    tf2.rotate(Eigen::AngleAxisd(90*M_PI/180, Eigen::Vector3d(1,0,0)));

    cout << tf1 * tf2 << endl;

    Translation vec(0, 0, 0);

    cout << endl << (tf1 * tf2 * vec).transpose() << endl;

    KinTransform ktf(tf2, Frame::World());

    cout << endl << tf1 * ktf << endl;

    vec = Translation(3, 2, 1);

    Axis test_axis;
    test_axis = vec;
    cout << endl << vec.transpose() << " -> " << test_axis.transpose() << endl;

    cout << endl << (tf1 * test_axis).transpose() << endl;
    cout << (tf2 * test_axis).transpose() << endl;

}

void test_rotations()
{
    Eigen::Quaterniond rot(Eigen::AngleAxisd(90*M_PI/180, Eigen::Vector3d(1,0,0)));

    Rotation otherrot = Eigen::AngleAxisd(1, Eigen::Vector3d(1,1,1));

    cout << rot.matrix() << endl;

}

int main(int argc, char* argv[])
{
    test_operators();

//    test_rotations();

}
