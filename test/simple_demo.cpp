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

#include "AkinIncludes.h"

using namespace akin;
using namespace std;

void run_demo()
{
    Frame base_frame(Frame::World(), "base_frame");
    
    Frame frame_A(base_frame, "A");
    Frame frame_B(base_frame, "B");
    Frame frame_C(frame_B, "C");
    
    frame_A.respectToRef( Transform( Translation(0, 1, 0) ) );
    frame_B.respectToRef( Transform( Translation(0,-1, 0),
                                     Rotation(90.0*M_PI/180.0, Axis(1, 0, 0)) ) );

    
    frame_C.respectToRef( Transform( Translation(3, 0, 0) ) );

    cout << frame_A << endl;
    cout << frame_B << endl;
    cout << frame_C << endl;
    
    cout << "Frame C wrt A:" << endl << frame_C.withRespectTo(frame_A) << endl;
    
    base_frame.respectToRef( Transform( Translation(0, 0, 10) ));
    cout << "\n\nNOW AFTER MOVING THE BASE FRAME: " << endl << endl;
    
    cout << frame_A << endl;
    cout << frame_B << endl;
    cout << frame_C << endl;
    
    cout << "Frame C wrt A:" << endl << frame_C.withRespectTo(frame_A) << endl;
    
    /// Creating a second kinematic tree
    
    Frame other_base(Frame::World(), "other_base");
    Frame A_prime(other_base, "A_prime");
    Frame B_prime(other_base, "B_prime");
    Frame C_prime(B_prime, "C_prime");
    
    A_prime.respectToRef( frame_A.respectToRef() );
    B_prime.respectToRef( frame_B.respectToRef() );
    C_prime.respectToRef( frame_C.respectToRef() );
    
    other_base.respectToRef( Transform( Translation(0, 0, -10) ) );
    
    cout << "\n\nCREATED A NEW VERSION OF THE FRAMES IN A DIFFERENT BASE FRAME:" << endl;
    
    cout << "\nA -> A_prime:\n" << A_prime.withRespectTo(frame_A) << endl;
    cout << "B -> B_prime:\n" << B_prime.withRespectTo(frame_B) << endl;
    cout << "C -> C_prime:\n" << C_prime.withRespectTo(frame_C) << endl;
    
    other_base.respectToRef( Transform( Translation(0, 0, -10),
                                        Rotation(90*M_PI/180, Axis(0, 1, 0)) ) );
    cout << "\n\nNOW AFTER ROTATING THE PRIMES' BASE FRAME:" << endl;
    
    cout << "\n\nA -> A_prime:\n" << A_prime.withRespectTo(frame_A) << endl;
    cout << "B -> B_prime:\n" << B_prime.withRespectTo(frame_B) << endl;
    cout << "C -> C_prime:\n" << C_prime.withRespectTo(frame_C) << endl;
    
    KinTranslation C_origin(Translation(0,0,0), frame_C, "C_origin");
    
    cout << C_origin << endl;
    cout << "The origin of C wrt A: " << C_origin.withRespectTo(frame_A).transpose() << endl;
    cout << "The origin of C wrt B: " << C_origin.withRespectTo(frame_B).transpose() << endl;
    cout << "The origin of C wrt C: " << C_origin.withRespectTo(frame_C).transpose() << endl;
}





int main(int argc, char* argv[])
{
    run_demo();
    
    return 0;
}
