
#include "Transform.h"
#include "Frame.h"

using namespace akin;
using namespace std;

int main(int argc, char* argv[])
{
    Transform testTransform;
    
    cout << testTransform << endl;
    
    cout << "\n";
    
    KinTransform ktf(Frame::World());
    cout << ktf << endl;
    
    testTransform.translate(Eigen::Vector3d(20, 1, 0));
    KinTransform altTf(Frame::World(), testTransform, "alt_constructed");
    
    cout << altTf << endl;
}
