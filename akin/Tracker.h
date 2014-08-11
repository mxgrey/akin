#ifndef AKIN_TRACKER_H
#define AKIN_TRACKER_H

#include "KinObject.h"

namespace akin {

class Tracker : public KinObject
{
public:
    
    KinCustomMacro( Tracker )
    
    Tracker(const std::string& name="tracker",
            verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    Tracker(Frame& target, const std::string& name="tracker",
            verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
    void clearNotification();
    
};

class MultiTracker : public Tracker
{
public:
    
    MultiTracker(const std::string& name="multitracker",
            verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
    MultiTracker(const MultiTracker& copyMultiTracker);
    MultiTracker& operator=(const MultiTracker& copyMultiTracker);
    
    void setSubscriptions(const KinObjectPtrArray& objects, bool notify=true);
    void addSubscriptions(const KinObjectPtrArray& objects, bool notify=true);
    void addSubscription(KinObject* object, bool notify=true);
    
    bool removeSubscription(KinObject* object, bool notify=true);
    void clearSubscriptions(bool notify=true);
    
protected:
    
    void _loseParent(KinObject* parent);
    KinObjectPtrArray _subscriptions;
    
    void _kinitialize(const MultiTracker& copy);
    
};

typedef std::vector<Tracker*> TrackerPtrArray;

} // namespace akin

#endif // AKIN_TRACKER_H
