
#include "Frame.h"

using namespace akin;
using namespace std;

void test_KinMacro()
{
    Frame baseFrame(Frame::World(), "baseFrame", verbosity::DEBUG);
    Frame childFrame(baseFrame, "childFrame");
    Frame newFrame(childFrame);

    KinTransform someTransform(childFrame, "first_transform");
    KinTransform otherTransform(someTransform);

    {
        KinTransform destructiveTransform(baseFrame, "destructo-matic");
    }

    cout << baseFrame << endl;
    cout << childFrame << endl;
    cout << newFrame << endl;
    cout << someTransform << endl;
    cout << otherTransform << endl;

}



int main(int argc, char* argv[])
{
    test_KinMacro();

    return 0;
}
