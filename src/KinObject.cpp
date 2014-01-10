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

    referenceFrame._gainChildObject(this);
    _referenceFrame = &referenceFrame;
    notifyUpdate();
}

KinObject::KinObject(const KinObject &other)
{
    _copyValues(other);
}

KinObject& KinObject::operator =(const KinObject& other)
{
    verb.debug() << "Assigning object '" << name() << "' to have the values of '" << other.name() << "'";

    _copyValues(other);
}

void KinObject::_copyValues(const KinObject &other)
{
    verb.debug() << "Copying object '" << other.name() << "' which is a " << other.type() << "'";

    verb.level = other.verb.level;
    name(other.name());
    _type = other.type();

    other.refFrame()._gainChildObject(this);
    _referenceFrame = &(other.refFrame());
    notifyUpdate();
}

KinObject& KinObject::Generic()
{
    static KinObject generic(Frame::World(), "generic",
                             verbosity::INHERIT, "Placeholder");
    return generic;
}

KinObject::~KinObject()
{
    refFrame()._loseChildObject(this);
}

Frame& KinObject::refFrame() const { return *_referenceFrame; }
std::string KinObject::name() const { return _name; }
void KinObject::name(std::string newName) { _name = newName; }

std::string KinObject::type() const { return _type; }

bool KinObject::changeRefFrame(Frame &newRefFrame)
{
    verb.desc() << "Changing the reference frame of " << name() << " from "
                << refFrame().name() << " to " << newRefFrame.name();
    verb.end();

    refFrame()._loseChildObject(this);
    newRefFrame._gainChildObject(this);

    _referenceFrame = &newRefFrame;
}

bool KinObject::descendsFrom(const Frame &someFrame)
{
    Frame* descentCheck = &refFrame();
    while(!descentCheck->isWorld())
    {
        if(&descentCheck->refFrame() == &someFrame)
            return true;
        descentCheck = &descentCheck->refFrame();
    }

    return false;
}

std::ostream& operator<<(std::ostream& oStrStream, const KinObject& mObject)
{
    oStrStream << mObject.type()
               << " named '" << mObject.name()
               << "' in frame '" << mObject.refFrame().name() << "'";
    return oStrStream;
}

void KinObject::notifyUpdate() { _needsUpdate = true; }
bool KinObject::needsUpdate() { return _needsUpdate; }

void KinObject::_loseParent()
{
    _referenceFrame = &Frame::World();
}
