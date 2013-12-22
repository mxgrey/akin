
#include "Frame.h"

using namespace akin;
using namespace std;

int main(int argc, char* argv[])
{
//    KinObject testObject(Frame::World(), "test object", "test type");
    Frame testFrame(Frame::World(), "test frame", verbosity::DEBUG);
    cout << "Test frame: " << testFrame.name() << endl;
    cout << "Ref frame: " << testFrame.refFrame().name() << endl;

    return 0;
}
