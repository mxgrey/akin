
#include "Frame.h"

using namespace akin;
using namespace std;

int main(int argc, char* argv[])
{
//    KinObject testObject(Frame::World(), "test object", "test type");
    Frame testFrame(Frame::World(), "test_frame", verbosity::DEBUG);
    cout << "Test frame: " << testFrame.name() << endl;
    cout << "Ref frame: " << testFrame.refFrame().name() << endl;

    Frame otherFrame(Frame::World(), "other_frame", verbosity::DEBUG);

    Frame newFrame(testFrame, "a_child_frame");
//    cout << "newFrame's parent: " << newFrame.refFrame().name() << endl;

    otherFrame.changeRefFrame(testFrame);
    newFrame.changeRefFrame(otherFrame);

    cout << endl;

    otherFrame.changeRefFrame(otherFrame);

    cout << endl;

    testFrame.changeRefFrame(newFrame);

    cout << endl;

    return 0;
}
