
#include "akin/Tracker.h"
#include "akin/Frame.h"

using namespace akin;
using namespace std;

Tracker::Tracker(const string &name, verbosity::verbosity_level_t report_level) :
    KinObject(Frame::World(), name, report_level, "Tracker")
{
    
}

void Tracker::_loseParent(KinObject *parent)
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

void Tracker::_kinitialize(const Tracker &copy)
{
    setSubscriptions(copy._subscriptions);
}

void Tracker::setSubscriptions(const KinObjectPtrArray& objects, bool notify)
{
    clearSubscriptions();
    addSubscriptions(objects);
    
    if(notify)
        notifyUpdate();
}

void Tracker::addSubscriptions(const KinObjectPtrArray &objects, bool notify)
{
    for(size_t i=0; i<objects.size(); ++i)
    {
        addSubscription(objects[i], false);
    }
    
    if(notify)
        notifyUpdate();
}

void Tracker::addSubscription(KinObject *object, bool notify)
{
    _subscriptions.push_back(object);
    _subscriptions.back()->_registerObject(this);
    
    if(notify)
        notifyUpdate();
}

bool Tracker::removeSubscription(KinObject *object, bool notify)
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

void Tracker::clearSubscriptions(bool notify)
{
    for(size_t i=0; i<_subscriptions.size(); ++i)
    {
        _subscriptions[i]->_unregisterObject(this);
    }
    
    _subscriptions.clear();
    
    if(notify)
        notifyUpdate();
}
