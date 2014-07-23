#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "akin/Body.h"
#include "akin/Tracker.h"

namespace akin {

class Robot;

class Manipulator : public Frame
{
public:
    
    Manipulator(Frame& referenceFrame, const std::string& manipName);
    
    void attachItem(Body& item);
    
    bool detachItem(Body& item);
    bool detachItem(size_t itemNum);
    bool detachItem(const std::string& name);
    
    bool deleteItem(Body& item);
    bool deleteItem(size_t itemNum);
    bool deleteItem(const std::string& name);
    
    
    
    const KinTranslation& com();
    
protected:
    
    Tracker _tracker;
    KinTranslation _com;
    
    BodyPtrArray _items;
    std::vector<Robot*> _robots;
    
    Robot* _myRobot;
};


} // namespace akin

#endif // MANIPULATOR_H
