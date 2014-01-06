
#include "Frame.h"

using namespace akin;

void printDifferentLevels(verbosity& verb)
{
    verb << "Printing default level"; verb.end();
    verb.log() << "Printing log level"; verb.end();
    verb.brief() << "Printing brief level"; verb.end();
    verb.desc() << "Printing descriptive level"; verb.end();
    verb.debug() << "Printing debug level"; verb.end();
}

int main(int argc, char* argv[])
{
    verbosity verb;

    std::cout << "Verbosity level after construction:" << std::endl;
    printDifferentLevels(verb);
    std::cout << std::endl;

    for(int i=0; i<verbosity::MAX_VERBOSITY_LEVEL; i++)
    {
        verb.level = verbosity::verbosity_level_t(i);
        std::cout << "Verbosity level on " << verb.level << ":" << std::endl;
        printDifferentLevels(verb);
        std::cout << std::endl;
    }

    verb.assertiveness = verbosity::ASSERT_NEVER;

    std::cout << "My assertiveness level is " << verb.assertiveness << "\n" << std::endl;

    std::cout << "Making a casual assertion" << std::endl;
    verb.Assert(false, verbosity::ASSERT_CASUAL,
                "This casual assertion should only get triggered if your assertiveness"
                "is set to ASSERT_CASUAL");
    std::cout << std::endl;

    std::cout << "Making a critical assertion" << std::endl;
    verb.Assert(false, verbosity::ASSERT_CRITICAL,
                "This critical assertion should always get triggered unless your"
                "assertiveness is set to ASSERT_NEVER");
    std::cout << std::endl;

    return 0;
}
