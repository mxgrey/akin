#ifndef TRACKER_H
#define TRACKER_H

#include "KinObject.h"

namespace akin {

class Tracker : public KinObject
{
public:
    
    KinCustomMacro( Tracker )
    
    Tracker(const std::string& name="tracker",
            verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
    void setSubscriptions(const KinObjectPtrArray& objects, bool notify=true);
    void addSubscriptions(const KinObjectPtrArray& objects, bool notify=true);
    void addSubscription(KinObject* object, bool notify=true);
    
    bool removeSubscription(KinObject* object, bool notify=true);
    void clearSubscriptions(bool notify=true);
    
protected:
    
    void _loseParent(KinObject* parent);
    KinObjectPtrArray _subscriptions;
    
};

} // namespace akin

#endif // TRACKER_H
