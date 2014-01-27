
#include "Joint.h"

using namespace akin;
using namespace std;

Joint::Joint(size_t jointID, const string &jointName,
             Link *mParentLink,
             const Transform &mBaseTransform,
             const Axis &mJointAxis, Type mType,
             double mininumValue, double maximumValue) :
    _id(jointID),
    _name(jointName),
    _parentLink(mParentLink),
    _childLink(mChildLink),
    _baseTransform(mBaseTransform),
    _axis(mJointAxis),
    _value(0),
    _min(mininumValue),
    _max(maximumValue),
    _myType(mType)
{

}


