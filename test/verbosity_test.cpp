
#include "Frame.h"

using namespace akin;

void printDifferentLevels(verbosity& verb)
{
    verb << "This is default level" << verb.end();
    verb.log() << "This is log level" << verb.end();
    verb.brief() << "This is brief level" << verb.end();
    verb.desc() << "This is descriptive level" << verb.end();
    verb.debug() << "This is debug level" << verb.end();
}

int main(int argc, char* argv[])
{
    verbosity verb;

    verb.level = verbosity::DEBUG;
    std::cout << "Verbosity level on debug:" << std::endl;
    printDifferentLevels(verb);

    verb.level = verbosity::BRIEF;
    std::cout << "Verbosity level on descriptive:" << std::endl;
    printDifferentLevels(verb);


    return 0;
}
