#ifndef HUBOKIN_HUBOLEGIK_H
#define HUBOKIN_HUBOLEGIK_H

#include "akin/AnalyticalIKBase.h"

namespace HuboKin {

template<int Q>
class HuboLegIK : public akin::AnalyticalIKTemplate<Q>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~HuboLegIK() { }

    typedef typename akin::Constraint<Q>::VectorQ VectorQ;

    HuboLegIK() { }
    HuboLegIK(int cspace_size) :
        akin::AnalyticalIKTemplate<Q>(cspace_size) { }
    HuboLegIK(akin::Manipulator& manipulator, const std::vector<size_t>& joints) :
        akin::RobotConstraintBase(manipulator.parentRobot(), joints),
        akin::ManipConstraintBase(manipulator),
        akin::AnalyticalIKTemplate<Q>(manipulator, joints)
    { _reconfigure(); }
    HuboLegIK(int cspace_size, akin::Manipulator& manipulator,
                            const std::vector<size_t>& joints) :
        akin::RobotConstraintBase(manipulator.parentRobot(), joints), 
        akin::ManipConstraintBase(manipulator),
        akin::AnalyticalIKTemplate<Q>(cspace_size, manipulator, joints) 
    { _reconfigure(); }


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

        const akin::Link& baseLink = this->_robot->joint(this->_joints[0]).const_upstreamLink();
        
        B = (baseLink.respectToWorld()*waist).inverse()
            * this->_goalTf.respectToWorld() * footRotationInv;
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
                q6 = akin::wrapToPi(q6+M_PI);

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
                q1 = akin::wrapToPi(q1+M_PI);

            q345 = atan2(-az/C2, -(C6*ax - S6*ay)/C2);
            q3 = akin::wrapToPi(q345 - q4 - q5);

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
    }

    double zeroSize;

protected:

    double L4, L5, L6;
    double nx, ny, nz, sx, sy, sz, ax, ay, az, px, py, pz;
    double q1, q2, q3, q4, q5, q6;
    double S2, S4, S6;
    double C2, C4, C5, C6;
    double C45, psi, q345;
    std::complex<double> radical;
    std::complex<double> sqrt_radical;
    akin::Transform B, Binv;
    akin::Transform waist;
    akin::Transform hipRotation;
    akin::Transform footRotationInv;
    Eigen::Matrix<int, 8, 3> alternatives;
    VectorQ testQ;

    virtual bool _reconfigure() {
        if(!akin::ManipConstraintBase::_reconfigure())
            return false;
        
        zeroSize = 1e-6;

        L4 = fabs(this->_robot->joint(this->_joints[3]).childLink().respectToRef().translation()[2]);
        L5 = fabs(this->_robot->joint(this->_joints[4]).childLink().respectToRef().translation()[2]);
        L6 = fabs(this->_manip->withRespectTo(this->_robot->joint(this->_joints[5]).childLink()).translation()[2]);

        hipRotation = Eigen::Isometry3d::Identity();
        hipRotation.rotate(akin::Rotation(90*akin::DEG, akin::Vec3(0,0,1)));
        
        waist =  this->_robot->joint(this->_joints[0]).baseTransform()
                  *this->_robot->joint(this->_joints[1]).baseTransform()
                  *this->_robot->joint(this->_joints[2]).baseTransform()
                  *hipRotation;

        footRotationInv = Eigen::Isometry3d::Identity();
        footRotationInv.rotate(akin::Rotation(-90*akin::DEG, akin::Vec3(0,1,0)));
        footRotationInv = footRotationInv.inverse();

        alternatives <<
                 1,  1,  1,
                 1,  1, -1,
                 1, -1,  1,
                 1, -1, -1,
                -1,  1,  1,
                -1,  1, -1,
                -1, -1,  1,
                -1, -1, -1;

        return true;
    }

};


} // namespace HuboKin

#endif // HUBOKIN_HUBOLEGIK_H
