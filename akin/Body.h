#ifndef AKIN_BODY_H
#define AKIN_BODY_H

#include "akin/Frame.h"

namespace akin {

class BodyBase
{
public:

    // Fill with pure abstract functions like getCom(), getMass(), getForces()
    //
    // This class will be inherited by both Body and Robot so that either can
    // be attached to a manipulator using the same API.

};

class Body : public Frame
{
public:
    
    Body(Frame& referenceFrame, const std::string& bodyName);
    
    KinTranslation com;
    double mass;
    
protected:
    
};

typedef std::vector<Body*> BodyPtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, const akin::Body& someBody);

#endif // AKIN_BODY_H
