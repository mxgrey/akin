#ifndef AKIN_MANIPULATOR_H
#define AKIN_MANIPULATOR_H

#include "akin/Body.h"

namespace akin {

class Robot;
class Link;
class ManipConstraintBase;
class AnalyticalIKBase;
class RobotSolverX;


class Manipulator : public Frame
{
public:
    
    friend class Robot;
    
    typedef enum {
        
        FREE=0,
        LINKAGE,
        FULLBODY,
        ANALYTICAL,
        SUPPORT,
        CUSTOM,
        
        NUM_MODES
        
    } Mode;

    static std::string mode_to_string(Mode m);

    bool ik(Eigen::VectorXd &config, const Transform &targetTf,
            Frame &relativeFrame=Frame::World());
    bool ik(Mode m, Eigen::VectorXd& config, const Transform& targetTf,
            Frame& relativeFrame=Frame::World());
    bool ik(Eigen::VectorXd &config, KinTransform &targetTf);
    bool ik(Mode m, Eigen::VectorXd& config, KinTransform& targetTf);
    
    std::vector<KinTranslation> supportGeometry;
    Mode mode;

    ManipConstraintBase* constraint();
    ManipConstraintBase* constraint(Mode m);
    RobotSolverX& solver(Mode m);
    AnalyticalIKBase& analyticalIK();
    void setConstraint(Mode m, ManipConstraintBase* newConstraint, bool giveOwnership=true);
    void resetConstraint(Mode m);
    void resetConstraints();
    
    const KinTranslation& point() const;
    
    int attachItem(Body* newItem);
    
    Body* item(size_t itemNum);
    const Body* item(size_t itemNum) const;
    size_t numItems() const;
    
    bool detachItem(Body* item);
    bool detachItem(size_t itemNum);
    
    bool deleteItem(Body* item);
    bool deleteItem(size_t itemNum);

    int attachRobot(Robot* newRobot);
    
    Robot* robot(size_t robotNum);
    const Robot* robot(size_t robotNum) const;
    size_t numRobots() const;
    
    // TODO: Consider what happens when an attached object gets deleted from elsewhere
    //          could be a good application for the Tracker class
    bool detachRobot(Robot* robot);
    bool detachRobot(size_t robotNum);
    
    Translation com(const Frame& withRespectToFrame = Frame::World()) const;
    const double& mass() const;
    
    Robot& parentRobot();
    const Robot& parentRobot() const;
    
    Link& parentLink();
    const Link& parentLink() const;
    
    bool changeRefFrame(Frame &newRefFrame);
    
    bool isDummy() const;
    
protected:
    
    Manipulator(Robot* robot, Frame& referenceFrame, const std::string& manipName,
                bool isDummy=false);

    virtual ~Manipulator();
    
    bool _ownConstraint[NUM_MODES];
    ManipConstraintBase* _constraints[NUM_MODES];
    RobotSolverX* _solvers[NUM_MODES];

    AnalyticalIKBase* _analytical;
    KinTranslation _point;
    mutable KinTranslation _com;
    mutable double _mass;
    
    BodyPtrArray _items;
    std::vector<Robot*> _robots;
    
    void _findParentLink();
    
    bool _isDummy;
    
    Link* _myLink;
    Robot* _myRobot;
};

typedef std::vector<Manipulator*> ManipPtrArray;

} // namespace akin

std::ostream& operator <<(std::ostream& stream, const akin::Manipulator::Mode& m);

#endif // AKIN_MANIPULATOR_H
