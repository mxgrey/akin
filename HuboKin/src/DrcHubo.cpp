
#include "../DrcHubo.h"

using namespace akin;
using namespace HuboKin;

DrcHubo::DrcHubo(const std::string &urdf_file,
                 const std::string &urdf_package_directory,
                 Frame &referenceFrame) :
    Hubo2Plus(urdf_file, urdf_package_directory, referenceFrame)
{
    _setup_pegs();
}

DrcHubo::DrcHubo(const std::string &urdf_string, Frame &referenceFrame) :
    Hubo2Plus(urdf_string, referenceFrame)
{
    _setup_pegs();
}

void DrcHubo::_setup_pegs()
{

}
