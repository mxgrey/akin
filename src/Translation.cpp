
#include "Translation.h"
#include "Frame.h"


using namespace std;
using namespace akin;

KinTranslation::KinTranslation(Frame &referenceFrame, string translationName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, translationName, report_level, "Translation"),
    Translation()
{

}

KinTranslation::KinTranslation(const Translation &relativeTranslation, Frame &referenceFrame, string translationName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, translationName, report_level, "Translation"),
    Translation(relativeTranslation)
{

}

const Translation& KinTranslation::respectToWorld()
{
    if(_needsUpdate)
        _update();

    return _respectToWorld;
}

Translation KinTranslation::withRespectTo(Frame &someFrame)
{
    return someFrame.respectToWorld().inverse() * respectToWorld();
}

void KinTranslation::_update()
{
    verb.debug() << "Updating translation '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * (Translation&)(*this);
}
