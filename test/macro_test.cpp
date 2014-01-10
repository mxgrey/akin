
#include "AkinIncludes.h"

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



int main(int argc, char* argv[])
{
    test_KinMacro();

    return 0;
}
