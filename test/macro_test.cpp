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

void test_KinMacro()
{
    Transform tf(Translation(1, 2, 3));
    
    Frame baseFrame(Frame::World(), "baseFrame", verbosity::DEBUG);
    
    cout << baseFrame << endl;
    
    Frame childFrame(tf, baseFrame, "childFrame");
    Frame newFrame(childFrame);

    KinTransform someTransform(childFrame, "first_transform");
    KinTransform otherTransform(someTransform);

    {
        KinTransform destructiveTransform(baseFrame, "destructo-matic");
        Frame destructiveFrame(childFrame, "destructorama");
        otherTransform.changeRefFrame(destructiveFrame);
        destructiveTransform.changeRefFrame(destructiveFrame);
    }
    

    cout << baseFrame << endl;
    cout << childFrame << endl;
    cout << newFrame << endl;
    cout << someTransform << endl;
    
    
    Transform baseTf(Translation(0, 0, 0), Rotation(90*M_PI/180,Axis(0,1,0)));
    baseFrame.respectToRef(baseTf);
    
    cout << someTransform << endl;

    baseFrame.respectToRef(Transform(Translation(1, 1, 1),
                                     Rotation(-90*M_PI/180,Axis(0,1,0))));
    
    cout << someTransform << endl;
}



int main(int , char* [])
{
    test_KinMacro();

    return 0;
}
