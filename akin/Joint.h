#ifndef JOINT_H
#define JOINT_H

#include "Frame.h"

namespace akin {

class Link;
class Robot;

class Joint
{
public:

    friend class Link;
    friend class Robot;

    typedef enum {
        DUMMY = 0,
        REVOLUTE,
        PRISMATIC,
        CUSTOM,

        JOINT_TYPE_SIZE
    } Type;

    /*!
     * \fn value(double newJointValue)
     * \brief Set this joint's value
     * \param newJointValue
     */
    virtual bool value(double newJointValue);

    /*!
     * \fn value()
     * \brief Get this joint's value
     * \return
     */
    inline virtual double value() { return _value; }

    /*!
     * \fn min(double newMinValue)
     * \brief Set this joint's minimum value
     * \param newMinValue
     */
    virtual bool min(double newMinValue);

    /*!
     * \fn min()
     * \brief Get this joint's minimum value
     * \return This joint's minimum value
     */
    inline virtual double min() { return _min; }

    /*!
     * \fn max(double newMaxValue)
     * \brief Set this joint's maximum value
     * \param newMaxValue
     */
    virtual bool max(double newMaxValue);

    /*!
     * \fn max()
     * \brief Get this joint's maximum value
     * \return This joint's maximum value
     */
    inline virtual double max() { return _max; }
    
    virtual bool withinLimits();
    virtual bool withinLimits(double someValue);

    /*!
     * \fn jointType()
     * \brief Returns this joint's type
     * \return This joint's type
     */
    inline Type jointType() { return _myType; }
    
    inline void jointType(Type newType)
    {
        _myType = newType; _computeRefTransform();
    }

    /*!
     * \fn jointAxis(Axis& jointAxis)
     * \brief Set the axis of this joint
     * \param newAxis
     */
    inline void jointAxis(Axis newAxis)
    {
        _axis = newAxis; _computeRefTransform();
    }

    /*!
     * \fn jointAxis()
     * \brief Get the axis of this joint
     * \return This joint's axis
     */
    inline const Axis& jointAxis() { return _axis; }

    /*!
     * \fn baseTransform(const Transform& newBaseTf)
     * \brief Set what the relative transform is when the joint value is 0.
     * \param newBaseTf
     */
    inline void baseTransform(const Transform& newBaseTf)
    {
        _baseTransform = newBaseTf; _computeRefTransform();
    }

    /*!
     * \fn baseTransform()
     * \brief Get what the relative transform is when the joint value is 0.
     * \return
     */
    inline const Transform& baseTransform() { return _baseTransform; }

    /*!
     * \fn id()
     * \brief Get this joint's index ID
     * \return
     */
    inline size_t id() { return _id; }

    /*!
     * \fn name()
     * \brief Get this joint's name
     * \return
     */
    inline std::string name() const { return _name; }

    /*!
     * \fn name(const std::string& new_name)
     * \brief Allows you to change the name of this string
     * \param new_name
     * \return
     *
     * This function can be used to modify a joint's name. If
     * new_name is already the name of a different joint, this function
     * will return false, and this joint's name will not be changed.
     * If this function returns true, then the name of this joint
     * was successfully changed.
     */
    virtual bool name(const std::string& new_name);


//    virtual void parentLink(Link& new_parent_link);
    inline Link& parentLink() { return *_parentLink; }
    inline Link& childLink() { return *_childLink; }
    
    Joint& parentJoint();
    Joint& childJoint(size_t num);
    size_t numChildJoints();
    
    inline Link& upstreamLink() { return *_upstreamLink; }
    inline Link& downstreamLink() { return *_downstreamLink; }
    
    Joint& upstreamJoint();
    Joint& downstreamJoint(size_t num);
    size_t numDownstreamJoints();

    inline bool belongsTo(const Robot& someRobot) const { return &someRobot == _myRobot; }

    inline bool isDummy() { return _myType == DUMMY; }
    
    inline Robot& robot() const { return *_myRobot; }
    
    verbosity verb;

protected:
    
    Joint(Robot* mRobot, size_t jointID=0, const std::string& jointName="joint",
          Link* mParentLink=NULL, Link* mChildLink=NULL,
          const Transform& mBaseTransform = Transform::Identity(),
          const Axis& mJointAxis = Axis(0, 0, 1), Joint::Type mType = Joint::REVOLUTE,
          double mininumValue=-M_PI, double maximumValue=M_PI,
          verbosity::verbosity_level_t report_level = verbosity::INHERIT);
    
    void _computeRefTransform();

    size_t _id;
    std::string _name;
    
    void _changeParentLink(Link* newParent);

    Link* _parentLink;
    Link* _childLink;

    bool _reversed;
    Link* _upstreamLink;
    Link* _downstreamLink;

    void _reverse();

    Transform _baseTransform;
    Axis _axis;

    double _value;
    double _min;
    double _max;

    Type _myType;

    Robot* _myRobot;
    
    ~Joint();
};

} // namespace akin


#endif // JOINT_H
