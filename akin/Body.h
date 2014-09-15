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

    typedef enum {

        LINEAR,
        ANGULAR

    } coord_t;
    
    Body(Frame& referenceFrame, const std::string& bodyName);
    
    KinTranslation com;
    double mass;


    // TODO: Give a lot of thought to this API
    // TODO: Move these velocity-related things into the Frame class
    virtual const KinVelocity& linearVelocity() const;
    virtual Velocity relativeLinearVelocity() const;
    virtual void relativelinearVelocity(const Velocity& v);

    virtual const KinVelocity& angularVelocity() const;
    virtual Velocity relativeAngularVelocity() const;
    virtual void relativeAngularVelocity(const Velocity& w);

    virtual const KinVelocity& velocity(coord_t type = LINEAR) const;
    virtual void velocity(const Velocity& v, coord_t type = LINEAR);
    virtual void velocity(const Screw& v);
    
protected:
    
    KinVelocity _linearV;
    KinVelocity _angularV;
    
};

typedef std::vector<Body*> BodyPtrArray;

} // namespace akin

std::ostream& operator<<(std::ostream& oStrStream, const akin::Body& someBody);

#endif // AKIN_BODY_H
