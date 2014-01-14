
#include "GraphicsWindow.h"

using namespace akin;

int main(int argc, char* argv[])
{
    GraphicsWindow wmgr(argc, argv, "test_window", 800, 600,
                        verbosity::DEBUG);

//    Frame dummyFrame(Transform(Translation(0, 0, 10)),
//                     Frame::World(), "dummy_frame", verbosity::DEBUG);
//    Box dummyBox(dummyFrame, "dummy_box", verbosity::DEBUG);
    Box dummyBox(Frame::World(), "dummy_box", verbosity::DEBUG);
    dummyBox.dimensions(3, 0.1, 0.2);
    dummyBox.color(1, 0, 0, 1);
    dummyBox.sideColor(Box::TOP, 0, 1, 0);
    dummyBox.sideColor(Box::LEFT, 0.7, 0, 0.7);
    GraphicsBuffer::storeGraphic(dummyBox);

    CheckGLError(wmgr.verb, "About to start running");
    wmgr.run();

    return 0;
}
