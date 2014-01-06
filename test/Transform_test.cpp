
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
}

int main(int argc, char* argv[])
{
    test_operators();

//    Transform testTransform;
    
//    cout << testTransform << endl;
    
//    cout << "\n";
    
//    KinTransform ktf(Frame::World());
//    cout << ktf << endl;
    
//    testTransform.translate(Eigen::Vector3d(20, 1, 0));
//    KinTransform altTf(testTransform, Frame::World(), "alt_constructed");
    
//    cout << altTf << endl;
}
