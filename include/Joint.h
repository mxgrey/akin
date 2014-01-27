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

    Joint(size_t jointID=0, const std::string& jointName="joint",
          Link* mParentLink=NULL,
          const Transform& mBaseTransform = Transform::Identity(),
          const Axis& mJointAxis = Axis(0, 0, 1), Joint::Type mType = Joint::REVOLUTE,
          double mininumValue=-M_PI, double maximumValue=M_PI);


    /*!
     * \fn value(double newJointValue)
     * \brief Set this joint's value
     * \param newJointValue
     */
    virtual void value(double newJointValue);

    /*!
     * \fn value()
     * \brief Get this joint's value
     * \return
     */
    virtual double value();

    /*!
     * \fn min(double newMinValue)
     * \brief Set this joint's minimum value
     * \param newMinValue
     */
    virtual void min(double newMinValue);

    /*!
     * \fn min()
     * \brief Get this joint's minimum value
     * \return This joint's minimum value
     */
    virtual double min();

    /*!
     * \fn max(double newMaxValue)
     * \brief Set this joint's maximum value
     * \param newMaxValue
     */
    virtual void max(double newMaxValue);

    /*!
     * \fn max()
     * \brief Get this joint's maximum value
     * \return This joint's maximum value
     */
    virtual double max();

    /*!
     * \fn getJointType()
     * \brief Returns this joint's type
     * \return This joint's type
     */
    Type getJointType();

    /*!
     * \fn jointAxis(Axis& jointAxis)
     * \brief Set the axis of this joint
     * \param newAxis
     */
    void jointAxis(Axis newAxis);

    /*!
     * \fn jointAxis()
     * \brief Get the axis of this joint
     * \return This joint's axis
     */
    const Axis& jointAxis();

    /*!
     * \fn baseTransform(const Transform& newBaseTf)
     * \brief Set what the relative transform is when the joint value is 0.
     * \param newBaseTf
     */
    void baseTransform(const Transform& newBaseTf);

    /*!
     * \fn baseTransform()
     * \brief Get what the relative transform is when the joint value is 0.
     * \return
     */
    const Transform& baseTransform();

    /*!
     * \fn id()
     * \brief Get this joint's index ID
     * \return
     */
    size_t id();

    /*!
     * \fn name()
     * \brief Get this joint's name
     * \return
     */
    std::string name();

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


    virtual void parentLink(Link& new_parent_link);
    Link& parentLink();

    Link& childLink();

    bool isDummy();

protected:

    size_t _id;
    std::string _name;

    Link* _parentLink;
    Link* _childLink;

    bool _reversed;
    Link* _streamChild;

    void _reverse();

    Transform _baseTransform;
    Axis _axis;

    double _value;
    double _min;
    double _max;

    Type _myType;
};

} // namespace akin


#endif // JOINT_H
