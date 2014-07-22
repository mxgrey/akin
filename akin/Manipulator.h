#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "akin/Body.h"

namespace akin {

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
    
    
    KinTranslation _com;
    BodyPtrArray _items;
    
};


} // namespace akin

#endif // MANIPULATOR_H
