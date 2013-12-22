#include "Frame.h"

using namespace akin;

Frame::Frame(Frame& referenceFrame, std::string frameName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, frameName, report_level, "Frame"),
    _isWorld(false)
{
}

Frame::Frame(bool createWorld) :
    _isWorld(true),
    KinObject(*this, "World", verbosity::DEBUG, "World Frame", true)
{

}

Frame& Frame::World()
{
    static Frame world(true);
    return world;
}

bool Frame::isWorld() { return _isWorld; }
