
#include "akin/Tracker.h"
#include "akin/Frame.h"

using namespace akin;
using namespace std;

Tracker::Tracker(const string &name, verbosity::verbosity_level_t report_level) :
    KinObject(Frame::World(), name, report_level, "Tracker")
{
    
}

Tracker::Tracker(Frame &target, const string &name, verbosity::verbosity_level_t report_level) :
    KinObject(target, name, report_level, "Tracker")
{
    
}

void Tracker::_kinitialize(const Tracker &)
{
    
}

void Tracker::clearNotification()
{
    _needsUpdate = false;
}

MultiTracker::MultiTracker(const string &name, verbosity::verbosity_level_t report_level) :
    Tracker(name, report_level)
{
    _type = "MultiTracker";
}

MultiTracker::MultiTracker(const MultiTracker &copyMultiTracker) :
    Tracker(copyMultiTracker)
{
    _kinitialize(copyMultiTracker);
}

MultiTracker& MultiTracker::operator =(const MultiTracker& copyMultiTracker)
{
    (Tracker&)(*this) = (Tracker&)(copyMultiTracker);
    _kinitialize(copyMultiTracker);
    notifyUpdate();
    return *this;
}

void MultiTracker::_loseParent(KinObject *parent)
{
    verb.debug() << "Removing '" << parent->name() << "' as a subscription of the Tracker '" 
                 << name() << "'";
    verb.end();
    
    int parentIndex = -1;
    for(size_t i=0; i<_subscriptions.size(); ++i)
    {
        if(_subscriptions[i] == parent)
        {
            parentIndex = i;
            break;
        }
    }
    
    if(parentIndex == -1)
    {
        verb.brief() << "Trying to remove '" << parent->name() << "' from the subscriptions of '" 
                     << name() << "', but it is not subscribed!";
        verb.desc() << " Children of '" << name() << "' include: ";
        for(size_t i=0; i<_subscriptions.size(); ++i)
            verb.desc() << " -- " << _subscriptions[i]->name() << "\n";
        verb.end();
        
        verb.Assert(false, verbosity::ASSERT_CASUAL, "");
    }
    else
    {
        _subscriptions.erase(_subscriptions.begin()+parentIndex);
    }
}

void MultiTracker::_kinitialize(const MultiTracker &copy)
{
    setSubscriptions(copy._subscriptions);
}

void MultiTracker::setSubscriptions(const KinObjectPtrArray& objects, bool notify)
{
    clearSubscriptions();
    addSubscriptions(objects);
    
    if(notify)
        notifyUpdate();
}

void MultiTracker::addSubscriptions(const KinObjectPtrArray &objects, bool notify)
{
    for(size_t i=0; i<objects.size(); ++i)
    {
        addSubscription(objects[i], false);
    }
    
    if(notify)
        notifyUpdate();
}

void MultiTracker::addSubscription(KinObject *object, bool notify)
{
    _subscriptions.push_back(object);
    _subscriptions.back()->_registerObject(this);
    
    if(notify)
        notifyUpdate();
}

bool MultiTracker::removeSubscription(KinObject *object, bool notify)
{
    if(object==NULL)
        return false;
    
    int subIndex = -1;
    for(size_t i=0; i<_subscriptions.size(); ++i)
    {
        if(_subscriptions[i] == object)
        {
            subIndex = i;
            break;
        }
    }
    
    if(subIndex == -1)
    {
        verb.brief() << "Trying to remove '" << object->name() << "' from the subscriptions of '" 
                     << name() << "', but it is not subscribed!";
        verb.desc() << " Subscriptions of '" << name() << "' include: ";
        for(size_t i=0; i<_subscriptions.size(); ++i)
            verb.desc() << " -- " << _subscriptions[i]->name() << "\n";
        verb.end();
        
        verb.Assert(false, verbosity::ASSERT_CASUAL, "");
        
        return false;
    }
    else
    {
        _subscriptions.erase(_subscriptions.begin()+subIndex);
    }
    
    if(notify)
        notifyUpdate();
    
    return true;
}

void MultiTracker::clearSubscriptions(bool notify)
{
    for(size_t i=0; i<_subscriptions.size(); ++i)
    {
        _subscriptions[i]->_unregisterObject(this);
    }
    
    _subscriptions.clear();
    
    if(notify)
        notifyUpdate();
}
