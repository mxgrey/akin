#ifndef AKIN_ANALYTICALIKBASE_H
#define AKIN_ANALYTICALIKBASE_H

#include "RobotConstraint.h"
#include "Solver.h"
#include <complex>

namespace akin {

class AnalyticalIKBase : public virtual ManipConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AnalyticalIKBase();
    
    AnalyticalIKBase();
    
    virtual void getSolutionsX(const Eigen::VectorXd& lastConfig,
                               std::vector<Eigen::VectorXd>& solutions,
                               std::vector<bool>& valid) = 0;
    virtual Validity getBestSolutionX(Eigen::VectorXd& best, const Eigen::VectorXd& lastConfig) = 0;
    
    int options;
    bool ignore_joint_limits;
    
};

class NullAnalyticalIK : public AnalyticalIKBase, public NullConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    void getSolutionsX(const Eigen::VectorXd&, std::vector<Eigen::VectorXd>& solutions,
                       std::vector<bool>& valid);
    Validity getBestSolutionX(Eigen::VectorXd& , const Eigen::VectorXd&);
    
};


template<int Q>
class AnalyticalIKTemplate : public ManipConstraint<Q>, public virtual AnalyticalIKBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AnalyticalIKTemplate() { }

    typedef typename ManipConstraint<Q>::VectorQ VectorQ;
    typedef typename ManipConstraint<Q>::Error Error;

    AnalyticalIKTemplate() : _goalTf(Frame::World(), "ik_goal") { _analyticalIKDefaults(); }
    AnalyticalIKTemplate(int cspace_size) :
        RobotConstraintBase(), AnalyticalIKBase(), ManipConstraint<Q>(cspace_size), 
        _goalTf(Frame::World(), "ik_goal") { _analyticalIKDefaults(); }
    AnalyticalIKTemplate(Manipulator& manipulator, const std::vector<size_t>& joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator),
        _goalTf(Frame::World(), manipulator.name()+"_ik_goal") { _analyticalIKDefaults(); }
    AnalyticalIKTemplate(int cspace_size, Manipulator& manipulator,
                        const std::vector<size_t>& joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator),
        ManipConstraint<Q>(cspace_size, manipulator, joints),
        _goalTf(Frame::World(), manipulator.name()+"_ik_goal") { _analyticalIKDefaults(); }

    virtual Validity getGradient(VectorQ& gradient, const VectorQ& configuration) {
        _tempBest = configuration;
        getError(configuration, this->computeErrorFromCenter);
        Validity v = this->_computeCurrentValidity();
        if(v.valid)
            return v;

        v = getBestSolution(_tempBest, configuration);
        gradient = configuration - _tempBest;
        if(v.stuck)
            return v;

        return Validity::Invalid();
    }

    virtual Validity getBestSolutionX(Eigen::VectorXd& best, const Eigen::VectorXd& lastConfig) {
        _tempBest = VectorQ(best); _tempLast = VectorQ(lastConfig);
        Validity v = getBestSolution(_tempBest, _tempLast);
        best = Eigen::VectorXd(_tempBest);
        return v;
    }

    virtual Validity getBestSolution(VectorQ& best, const VectorQ& lastConfig) {
        _solutions.clear(); _valid.clear();
        getSolutions(lastConfig, _solutions, _valid);
        size_t selection = selectBestSolution(best, lastConfig, _solutions, _valid);
        if(selection == DOF_INVALID)
            return Validity::Stuck();
        else if(_valid[selection])
            return Validity::Valid();

        return Validity::Stuck();
    }

    virtual void getSolutionsX(const Eigen::VectorXd& lastConfig,
                               std::vector<Eigen::VectorXd>& solutions,
                               std::vector<bool>& valid) {
        _solutions.clear(); valid.clear();
        getSolutions(VectorQ(lastConfig), _solutions, valid);
        solutions.resize(_solutions.size());
        for(size_t i=0; i<_solutions.size(); ++i)
            solutions[i] = Eigen::VectorXd(_solutions[i]);
    }

    virtual void getSolutions(const VectorQ& lastConfig,
                              std::vector<VectorQ>& solutions,
                              std::vector<bool>& valid) = 0;

    virtual size_t selectBestSolution(VectorQ& best, const VectorQ& lastConfig,
                                      const std::vector<VectorQ>& solutions,
                                      const std::vector<bool>& valid) {
        _validChoices.clear();
        for(size_t i=0; i<valid.size(); ++i)
            if(valid[i])
                _validChoices.push_back(i);

        bool valid_exists = _validChoices.size() > 0;
        size_t result = DOF_INVALID; double best_cost = INFINITY;
        double cost=0;
        if(valid_exists)
        {
            for(size_t v=0; v<_validChoices.size(); ++v)
            {
                const size_t& i=_validChoices[v];
                cost = rateConfigCost(solutions[i], lastConfig);
                if( cost < best_cost )
                {
                    result = i;
                    best_cost = cost;
                }
            }
        }
        else
        {
            for(size_t i=0; i<solutions.size(); ++i)
            {
                cost = rateConfigCost(solutions[i], lastConfig);
                if( cost < best_cost )
                {
                    result = i;
                    best_cost = cost;
                }
            }
        }

        if(result != DOF_INVALID)
            best = solutions[result];
        return result;
    }
    
    const KinTransform& getGoalTransform(const VectorQ& config) {
        getError(config, this->computeErrorFromCenter);
        _goalTf.changeRefFrame(this->target.refFrame());
        
        Error _target_disp = this->_displacement - this->_error;

        _goalTf = Eigen::Isometry3d::Identity();
        _goalTf.translate(_target_disp.template block<3,1>(0,0));
        _goalTf.translate(this->target.respectToRef().translation());
        _goalTf.rotate(Rotation(_target_disp[3], Vec3(1,0,0)));
        _goalTf.rotate(Rotation(_target_disp[4], Vec3(0,1,0)));
        _goalTf.rotate(Rotation(_target_disp[5], Vec3(0,0,1)));
        _goalTf.rotate(this->target.respectToRef().rotation());

        return _goalTf;
    }

    // TODO: Decide on a variety of criteria for deciding what is the best solution
    virtual double rateConfigCost(const VectorQ& config, const VectorQ& lastConfig) {
        return (config-lastConfig).norm();
    }

protected:

    void _analyticalIKDefaults() {
        this->error_weights.resize(6); error_weights.setOnes();
        this->min_limits.setZero(); this->max_limits.setZero();
        this->computeErrorFromCenter = false;
    }

    VectorQ _tempBest;
    VectorQ _tempLast;
    std::vector<VectorQ> _solutions;
    std::vector<bool> _valid;
    std::vector<size_t> _validChoices;
    KinTransform _goalTf;
    Transform _tf_target_disp;

};

typedef AnalyticalIKTemplate<Eigen::Dynamic> AnalyticalIKTemplateX;


template<int Q>
class AnalyticalBallSocketHip : public AnalyticalIKTemplate<Q>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AnalyticalBallSocketHip() { }

    typedef typename Constraint<Q>::VectorQ VectorQ;

    AnalyticalBallSocketHip() { }
    AnalyticalBallSocketHip(int cspace_size) :
        AnalyticalIKTemplate<Q>(cspace_size) { }
    AnalyticalBallSocketHip(Manipulator& manipulator, const std::vector<size_t>& joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator),
        AnalyticalIKTemplate<Q>(manipulator, joints)
    { _reconfigure(); }
    AnalyticalBallSocketHip(int cspace_size, Manipulator& manipulator,
                            const std::vector<size_t>& joints) :
        RobotConstraintBase(manipulator.parentRobot(), joints), ManipConstraintBase(manipulator),
        AnalyticalIKTemplate<Q>(cspace_size, manipulator, joints) { _reconfigure(); }


    virtual void getSolutions(const VectorQ& lastConfig,
                              std::vector<VectorQ>& solutions,
                              std::vector<bool>& valid) {
        for(int i=6; i<this->_config_size; ++i)
        {
            this->_robot->joint(this->_joints[i]).value(lastConfig[i]);
            testQ[i] = lastConfig[i];
        }
        solutions.clear();
        valid.clear();
        solutions.reserve(8);
        valid.reserve(8);
        getGoalTransform(lastConfig);

        const Link& baseLink = this->_robot->joint(this->_joints[0]).const_upstreamLink();
        
        B = (baseLink.respectToWorld()*waist*hipRotation).inverse() 
            * this->_goalTf.respectToWorld() * footRotation.inverse();
        Binv = B.inverse();

        nx = Binv(0,0); sx = Binv(0,1); ax = Binv(0,2); px = Binv(0,3);
        ny = Binv(1,0); sy = Binv(1,1); ay = Binv(1,2); py = Binv(1,3);
        nz = Binv(2,0); sz = Binv(2,1); az = Binv(2,2); pz = Binv(2,3);

        for(int i=0; i<8; ++i)
        {
            bool isValid = true;

            C4 = ((px+L6)*(px+L6) - L4*L4 - L5*L5 + py*py + pz*pz)/(2*L4*L5);
            radical = 1-C4*C4;
            sqrt_radical = std::sqrt(radical);
            if(sqrt_radical.imag() != 0)
                isValid = false;
            q4 = atan2(alternatives(i,0)*sqrt_radical.real(), C4);

            S4 = sin(q4);
            psi = atan2(S4*L4, C4*L4+L5);
            radical = (px+L6)*(px+L6) + py*py;
            sqrt_radical = std::sqrt(radical);
            if(sqrt_radical.imag() != 0)
                isValid = false;

            q5 = wrapToPi(atan2(-pz, alternatives(i,1)*sqrt_radical.real())-psi);

            q6 = atan2(py, -(px+L6));
            C45 = cos(q4+q5);
            C5 = cos(q5);
            if( C45*L4 + C5*L5 < 0 )
                q6 = wrapToPi(q6+M_PI);

            S6 = sin(q6);
            C6 = cos(q6);

            S2 = C6*ay + S6*ax;
            radical = 1-S2*S2;
            sqrt_radical = std::sqrt(radical);
            if(sqrt_radical.imag() != 0)
                isValid = false;
            q2 = atan2(S2, alternatives(i,2)*sqrt_radical.real());

            q1 = atan2(C6*sy + S6*sx, C6*ny + S6*nx);
            C2 = cos(q2);
            if( C2 < 0 )
                q1 = wrapToPi(q1+M_PI);

            q345 = atan2(-az/C2, -(C6*ax - S6*ay)/C2);
            q3 = wrapToPi(q345 - q4 - q5);

            testQ[0]=q1; testQ[1]=q2; testQ[2]=q3; testQ[3]=q4; testQ[4]=q5; testQ[5]=q6;
            
            for(int k=0; k<testQ.size(); ++k)
                if( fabs(testQ[k]) < zeroSize )
                    testQ[k] = 0;
            
            bool add_result = true;
            for(int j=0; j<6; ++j)
            {
                if(!this->_robot->joint(this->_joints[j]).withinLimits(testQ[j]))
                {
                    add_result = false;
                    break;
                }
            }

            if(add_result)
            {
                solutions.push_back(testQ);
                valid.push_back(isValid);
            }
        }

        // TODO: Check the stuff above
        // TODO: Look over solution selection
    }

    double zeroSize;
    Transform hipRotation;
    Transform footRotation;

protected:

    double L4, L5, L6;
    double nx, ny, nz, sx, sy, sz, ax, ay, az, px, py, pz;
    double q1, q2, q3, q4, q5, q6;
    double S2, S4, S6;
    double C2, C4, C5, C6;
    double C45, psi, q345;
    std::complex<double> radical;
    std::complex<double> sqrt_radical;
    Transform B, Binv;
    Transform waist;
    Eigen::Matrix<int, 8, 3> alternatives;
    VectorQ testQ;

    virtual bool _reconfigure() {
        if(!ManipConstraintBase::_reconfigure())
            return false;
        
        zeroSize = 1e-6;

        L4 = fabs(this->_robot->joint(this->_joints[3]).childLink().respectToRef().translation()[2]);
        L5 = fabs(this->_robot->joint(this->_joints[4]).childLink().respectToRef().translation()[2]);
        L6 = fabs(this->_manip->withRespectTo(this->_robot->joint(this->_joints[5]).childLink()).translation()[2]);
        
        waist =  this->_robot->joint(this->_joints[0]).baseTransform()
                  *this->_robot->joint(this->_joints[1]).baseTransform()
                  *this->_robot->joint(this->_joints[2]).baseTransform();

        alternatives <<
                 1,  1,  1,
                 1,  1, -1,
                 1, -1,  1,
                 1, -1, -1,
                -1,  1,  1,
                -1,  1, -1,
                -1, -1,  1,
                -1, -1, -1;

        hipRotation = Eigen::Isometry3d::Identity();
        footRotation = Eigen::Isometry3d::Identity();

        return true;
    }

};


} // namespace akin


#endif // AKIN_ANALYTICALIKBASE_H
