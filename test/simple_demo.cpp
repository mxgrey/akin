
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
