
#include "Transform.h"
#include "Frame.h"

using namespace akin;
using namespace std;

void test_operators()
{
    Transform tf1;
    tf1.translate(Eigen::Vector3d(10, 5, 1));

    Transform tf2;
    tf2.rotate(Eigen::AngleAxisd(90*M_PI/180, Eigen::Vector3d(1,0,0)));

    cout << tf1 * tf2 << endl;

    Translation vec(0, 0, 0);

    cout << endl << (tf1 * tf2 * vec).transpose() << endl;

    KinTransform ktf(tf2, Frame::World());

    cout << endl << tf1 * ktf << endl;

    vec = Translation(3, 2, 1);

    Axis test_axis;
    test_axis = vec;
    cout << endl << vec.transpose() << " -> " << test_axis.transpose() << endl;

    cout << endl << (tf1 * test_axis).transpose() << endl;
    cout << (tf2 * test_axis).transpose() << endl;

}

void test_rotations()
{
    Eigen::Quaterniond rot(Eigen::AngleAxisd(90*M_PI/180, Eigen::Vector3d(1,0,0)));

    cout << rot.matrix() << endl;

}

int main(int argc, char* argv[])
{
    test_operators();

//    test_rotations();

}
