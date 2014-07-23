#ifndef CENTEROFMASS_H
#define CENTEROFMASS_H

#include "Tracker.h"
#include "Body.h"

namespace akin {

class CenterOfMass : public Tracker
{
public:
    
    static Translation computeCoM(const BodyPtrArray& bodies);
    
    CenterOfMass(const std::string& name="com",
                 verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
    const KinTranslation& getCoM();
    double getMass();
    
    void addItem(Body* item);
    bool removeItem(Body* item);
    
    void addCoM(CenterOfMass* com);
    bool removeCoM(CenterOfMass* com);
    
protected:
    
    KinTranslation _com;
    double _mass;
    
    BodyPtrArray _bodies;
    
};

} // namespace akin

#endif // CENTEROFMASS_H
