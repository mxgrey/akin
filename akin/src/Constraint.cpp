
#include "akin/Constraint.h"
#include "sstream"

using namespace akin;
using namespace std;

ConstraintBase::Validity::Validity() :
    valid(false),
    stuck(false),
    near_edge(false)
{
    
}

std::string ConstraintBase::Validity::toString() const
{
    stringstream str;
    if(valid)
        str << "VALID";
    else
        str << "INVALID";
    
    if(stuck)
        str << "|STUCK";
    
    if(near_edge)
        str << "|NEAR EDGE";
    
    return str.str();
}
