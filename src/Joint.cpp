
#include "Joint.h"

using namespace akin;
using namespace std;

Joint::Joint(Robot *mRobot, size_t jointID, const string &jointName,
             Link *mParentLink, Link *mChildLink,
             const Transform &mBaseTransform,
             const Axis &mJointAxis, Type mType,
             double mininumValue, double maximumValue) :
    _myRobot(mRobot),
    _id(jointID),
    _name(jointName),
    _parentLink(mParentLink),
    _childLink(mChildLink),
    _baseTransform(mBaseTransform),
    _axis(mJointAxis),
    _min(mininumValue),
    _max(maximumValue),
    _myType(mType)
{
    value(0);
}


Joint::hasRobot() { return _myRobot != NULL; }


