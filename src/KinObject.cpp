#include "Frame.h"

using namespace akin;

KinObject::KinObject(Frame& referenceFrame,
                     std::string myName,
                     verbosity::verbosity_level_t report_level,
                     std::string myType, bool thisIsTheWorld) :
    verb(report_level)
{
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
