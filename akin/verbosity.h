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

#ifndef AKIN_VERBOSITY_H
#define AKIN_VERBOSITY_H

#include <string>
#include <iostream>
#include <stdlib.h>

namespace akin {

/*!
 * \class verbosity
 * \brief Verbosity management class
 *
 * The verbosity class manages what output goes to the screen based on user
 * settings. It also decides whether or not to quit based on user settings.
 */
class verbosity
{
public:

    /*!
     * \enum verbosity_level_t
     * \brief How verbose should the object be
     *
     * Verbosity level indicates what level of description should be provided
     * when things are printing out. Verbosity level can be changed in the
     * verbosity object at any time.
     */
    typedef enum {
        SILENT=0,
        LOG,
        INHERIT,
        BRIEF,
        DESCRIPTIVE,
        DEBUG,
        MAX_VERBOSITY_LEVEL
    } verbosity_level_t;
    verbosity_level_t level;

    static inline std::string verbosity_level_to_string(verbosity_level_t verb_level)
    {
        switch(verb_level)
        {
            case SILENT: return "SILENT"; break;
            case LOG: return "LOG"; break;
            case INHERIT: return "INHERIT"; break;
            case BRIEF: return "BRIEF"; break;
            case DESCRIPTIVE: return "DESCRIPTIVE"; break;
            case DEBUG: return "DEBUG"; break;
            case MAX_VERBOSITY_LEVEL: return "MAX_VERBOSITY_LEVEL"; break;
        }

        return "Invalid Verbosity Level Type";
    }

    /*!
     * \enum assertion_level_t
     * \brief How assertive should the object be
     *
     * Assertion level indicates how strict the object should be about casting
     * assertions. Assertions force the program to quit execution if a condition
     * is not met.
     */
    typedef enum {
        ASSERT_NEVER=0,
        ASSERT_CRITICAL,
        ASSERT_CASUAL
    } assertion_level_t;
    assertion_level_t assertiveness;

    static inline std::string assert_level_to_string(assertion_level_t assert_level)
    {
        switch(assert_level)
        {
            case ASSERT_NEVER: return "ASSERT_NEVER"; break;
            case ASSERT_CRITICAL: return "ASSERT_CRITICAL"; break;
            case ASSERT_CASUAL: return "ASSERT_CASUAL"; break;
        }
        
        return "";
    }

    inline verbosity(verbosity_level_t verb_level = LOG, std::ostream& targetStream = std::cout)
    {
        _outputstream = &targetStream;
        level = verb_level;
        _streamtype = verbosity::LOG;
        assertiveness = ASSERT_CRITICAL;
        buffer.clear();
    }

    /*!
     * \brief Get a reference to the verbosity object's output stream
     * \return The current output stream of the verbosity object
     */
    inline std::ostream& stream()
    {
        return *_outputstream;
    }
    /*!
     * \brief Specify the output stream
     */
    inline void stream(std::ostream& newStream)
    {
        _outputstream = &newStream;
    }

    // TODO: Look into how ostream handles many different types
    inline verbosity& operator<<(const std::string& message)
    {
        if(_streamtype <= level)
            buffer += message;
        return *this;
    }

    /*!
     * \brief Set the streaming mode to logging
     * \return
     */
    inline verbosity& log()
    {
        _streamtype = LOG;
        return *this;
    }

    /*!
     * \brief Set the streaming mode to brief
     * \return
     */
    inline verbosity& brief()
    {
        _streamtype = BRIEF;
        return *this;
    }

    /*!
     * \brief Set the streaming mode to descriptive
     * \return
     */
    inline verbosity& desc()
    {
        _streamtype = DESCRIPTIVE;
        return *this;
    }

    /*!
     * \brief Set the streaming mode to debug
     * \return
     */
    inline verbosity& debug()
    {
        _streamtype = DEBUG;
        return *this;
    }

    /*!
     *
     * \fn end()
     * \brief Tells the verbosity object to flush out the current message
    */
    inline verbosity& end()
    {
        if(buffer.size() > 0)
            *_outputstream << buffer << std::endl;
        buffer.clear();
        _streamtype = LOG; // TODO: Consider making this SILENT
        return *this;
    }

    inline verbosity& operator<<(const verbosity&)
    {
        return *this;
    }

    inline bool Assert(bool condition, assertion_level_t importance, const std::string& brief_explanation,
                       const std::string& desc_explanation = "")
    {
        if(!condition)
        {
            brief() << brief_explanation;
            desc() << desc_explanation;
            end();

            if( ASSERT_NEVER < importance && importance <= assertiveness)
            {
                *_outputstream << "ASSERTION LEVEL: " << assert_level_to_string(importance);
                *_outputstream << " | MY ASSERTIVENESS LEVEL: "
                               << assert_level_to_string(assertiveness) << std::endl;
                std::flush(*_outputstream);
                exit(1);
            }
        }

        return condition;
    }

    std::string buffer;

private:
    verbosity_level_t _streamtype;

    std::ostream* _outputstream;
};



inline std::ostream& operator<<(std::ostream& oStrStream,
                                const verbosity::verbosity_level_t verb_level)
{
    oStrStream << verbosity::verbosity_level_to_string(verb_level);
    return oStrStream;
}

inline std::ostream& operator<<(std::ostream& oStrStream,
                                const verbosity::assertion_level_t assert_level)
{
    oStrStream << verbosity::assert_level_to_string(assert_level);
    return oStrStream;
}

} // namespace akin


#endif // AKIN_VERBOSITY_H
