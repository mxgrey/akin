
#include "GraphicsWindow.h"

using namespace akin;

int main(int argc, char* argv[])
{
    GraphicsWindow wmgr(argc, argv, "test_window", 800, 600,
                        verbosity::DEBUG);


    CheckGLError(wmgr.verb, "About to start running");
    wmgr.run();

    return 0;
}
