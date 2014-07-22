#ifndef BODY_H
#define BODY_H

#include "akin/Frame.h"

namespace akin {

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


#endif // BODY_H
