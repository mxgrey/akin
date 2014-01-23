/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Jan 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Humanoid Robotics Lab nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "AkinIncludes.h"

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
