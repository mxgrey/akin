#ifndef AKIN_DEGREEOFFREEDOM_H
#define AKIN_DEGREEOFFREEDOM_H

#include "akin/Frame.h"

namespace akin {

class Joint;
class Robot;

class DofProperties
{
public:

    DofProperties(double minimumValue=-INFINITY, double maximumValue=INFINITY,
                  double maxSpeed=INFINITY, double maxAcceleration=INFINITY,
                  double maxTorque=INFINITY);

    double _value;
    double _minValue;
    double _maxValue;

    double _velocity;
    double _maxSpeed;

    double _acceleration;
    double _maxAcceleration;

    double _effort;
    double _maxEffort;

};

typedef std::vector<DofProperties> DofPropertyArray;

class DegreeOfFreedom : protected DofProperties
{
public:

    friend class Link;
    friend class Joint;
    friend class Robot;

    bool position(double newDofPosition);
    double position() const;

    bool velocity(double newDofVelocity);
    double velocity() const;

    bool acceleration(double newDofAcceleration);
    double acceleration() const;

    bool effort(double newDofEffort);
    double effort() const;

    bool property(property_t p, double newValue);
    double property(property_t p) const;

    void limits(double newMinValue, double newMaxValue);
    void limits(const std::pair<double,double>& newLimits);
    std::pair<double,double> limits() const;

    bool min(double newMinValue);
    double min() const;

    bool max(double newMaxValue);
    double max() const;

    void maxSpeed(double newMaxSpeed);
    double maxSpeed() const;

    void maxAcceleration(double newMaxAcceleration);
    double maxAcceleration() const;

    bool withinLimits() const;
    bool withinLimits(double someValue) const;




    Vec3 Jacobian_posOnly(const KinTranslation& point, const Frame& refFrame,
                          bool checkDependence=true) const;
    Vec3 Jacobian_rotOnly(const KinTranslation& point, const Frame& refFrame,
                          bool checkDependence=true) const;
    Screw Jacobian(const KinTranslation& point, const Frame& refFrame,
                   bool checkDependence=true) const;

    const std::string& name() const;
    bool name(const std::string& newName);

    Joint& joint();
    const Joint& joint() const;

    Robot& robot();
    const Robot& robot() const;

    size_t id() const;
    size_t localID() const; // TODO: Come up with a better name for this

    verbosity& verb;

protected:

    DegreeOfFreedom(Joint* parentJoint, const std::string& name, const DofProperties& properties);

    Vec3 _computePosJacobian(const Vec3& z_i, const KinTranslation& point,
                             const Frame& refFrame) const;
    Vec3 _computeRotJacobian(const Vec3& z_i, const Frame& refFrame) const;
    void _computeTransformedJointAxis(Vec3& z_i, const Frame& refFrame) const;

    size_t _id;
    size_t _localID;
    std::string _name;

    bool _isDummy;

    Joint* _parent;
    Robot* _robot;

};

typedef std::vector<DegreeOfFreedom*> DofPtrArray;

}




#endif // AKIN_DEGREEOFFREEDOM_H
