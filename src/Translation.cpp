
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

    _needsUpdate = false;
}

KinFreeVector::KinFreeVector(Frame &referenceFrame, string freeVectorName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, freeVectorName, report_level, "FreeVector"),
    FreeVector()
{

}

KinFreeVector::KinFreeVector(const FreeVector &relativeFreeVector, Frame &referenceFrame, string freeVectorName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, freeVectorName, report_level, "FreeVector"),
    FreeVector(relativeFreeVector)
{

}

const FreeVector& KinFreeVector::respectToWorld()
{
    if(_needsUpdate)
        _update();

    return _respectToWorld;
}

FreeVector KinFreeVector::withRespectTo(Frame &someFrame)
{
    // TODO: Investigate if it is okay to leave off Transform()
    return Transform(someFrame.respectToWorld().inverse()) * respectToWorld();
}

void KinFreeVector::_update()
{
    verb.debug() << "Updating FreeVector '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * (FreeVector&)(*this);

    _needsUpdate = false;
}

KinAxis::KinAxis(Frame &referenceFrame, string axisName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, axisName, report_level, "Axis"),
    Axis()
{

}

KinAxis::KinAxis(const Axis &relativeAxis, Frame &referenceFrame, string axisName, verbosity::verbosity_level_t report_level) :
    KinObject(referenceFrame, axisName, report_level, "Axis"),
    Axis(relativeAxis)
{

}


const Axis& KinAxis::respectToWorld()
{
    if(_needsUpdate)
        _update();

    return _respectToWorld;
}

Axis KinAxis::withRespectTo(Frame &someFrame)
{
    return Transform(someFrame.respectToWorld().inverse()) * respectToWorld();
}

void KinAxis::_update()
{
    verb.debug() << "Updating Axis '"+name()+"'"; verb.end();

    _respectToWorld = refFrame().respectToWorld() * (Axis&)(*this);

    _needsUpdate = false;
}
