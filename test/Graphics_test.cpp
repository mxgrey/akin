
#include "GraphicsWindow.h"

using namespace akin;

int main(int argc, char* argv[])
{
    GraphicsWindow wmgr(argc, argv, "test_window", 800, 600,
                        verbosity::DEBUG);

    wmgr.run();

    return 0;
}
