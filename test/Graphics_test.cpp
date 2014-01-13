
#include "GraphicsWindow.h"

using namespace akin;

int main(int argc, char* argv[])
{
    GraphicsWindow wmgr(argc, argv, "test_window", 800, 600,
                        verbosity::DEBUG);

//    Frame dummyFrame(Transform(Translation(0, 0, 10)),
//                     Frame::World(), "dummy_frame", verbosity::DEBUG);
//    Box dummyBox(dummyFrame, "dummy_box", verbosity::DEBUG);
//    dummyBox.dimensions(2, 2, 2);
//    GraphicsBuffer::displayGraphic(dummyBox);

    CheckGLError(wmgr.verb, "About to start running");
    wmgr.run();

    return 0;
}
