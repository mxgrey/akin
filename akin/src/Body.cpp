
#include "akin/Body.h"

using namespace akin;
using namespace std;

Body::Body(Frame &referenceFrame, const string &bodyName) :
    Frame(referenceFrame, bodyName),
    com(*this, bodyName+"_com"),
    mass(0)
{
    com.setZero();
}

Translation Body::getCom(const Frame &withRespectToFrame) const
{
    return com.withRespectTo(withRespectToFrame);
}

double Body::getMass() const
{
    return mass;
}



std::ostream& operator<<(std::ostream& oStrStream, const akin::Body& someBody)
{
    std::cout << "Body named '" << someBody.name() << "' has mass " << someBody.mass 
              << " and a relative Center of Mass <" << someBody.com.transpose() << ">\n";
    oStrStream << (akin::Frame&)someBody;
    return oStrStream;
}
