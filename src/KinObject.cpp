#include "Frame.h"

using namespace akin;

KinObject::KinObject(Frame& referenceFrame,
                     std::string myName,
                     verbosity::verbosity_level_t report_level,
                     std::string myType, bool thisIsTheWorld)
{
    if(report_level == verbosity::INHERIT)
        verb.level = referenceFrame.verb.level;
    else
        verb.level = report_level;

    verb.debug() << "Creating " << myName << " which is a " << myType;
    if(!thisIsTheWorld)
        verb << " in the frame of " << referenceFrame.name();
    verb.end();

    name(myName);
    _type = myType;

    _referenceFrame = &referenceFrame;
}

KinObject::~KinObject()
{

}

Frame& KinObject::refFrame() { return *_referenceFrame; }
std::string KinObject::name() { return _name; }
void KinObject::name(std::string newName) { _name = newName; }

void KinObject::changeRefFrame(Frame &newRefFrame)
{
    verb.desc() << "Changing the reference frame of " << name() << " from "
                << refFrame().name() << " to " << newRefFrame.name();
    verb.end();

    refFrame()._loseChildObject(this);
    newRefFrame._gainChildObject(this);

    _referenceFrame = &newRefFrame;
}
