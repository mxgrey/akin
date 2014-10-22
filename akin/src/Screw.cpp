
#include "../Screw.h"

Eigen::Matrix3d akin::skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d S;
    for(size_t i=0; i<3; ++i)
        S(i,i) = 0;

    S(0,1) = -v[2];
    S(0,2) =  v[1];

    S(1,0) =  v[2];
    S(1,2) = -v[0];

    S(2,0) = -v[1];
    S(2,1) =  v[0];

    return S;
}

akin::Matrix6d akin::spatial_transform(const Eigen::Isometry3d &tf)
{
    akin::Matrix6d X;
    X.block<3,3>(0,0) = tf.rotation();
    X.block<3,3>(0,3).setZero();
    X.block<3,3>(3,0) = -tf.rotation()*skew(tf.translation());
    X.block<3,3>(3,3) = tf.rotation();

    return X;
}

akin::Matrix6d akin::spatial_transform_star(const Eigen::Isometry3d &tf)
{
    akin::Matrix6d X;
    X.block<3,3>(0,0) = tf.rotation();
    X.block<3,3>(0,3) = -tf.rotation()*skew(tf.translation());
    X.block<3,3>(3,0).setZero();
    X.block<3,3>(3,3) = tf.rotation();

    return X;
}
