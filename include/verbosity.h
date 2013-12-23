#ifndef VERBOSITY_H
#define VERBOSITY_H

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
        BRIEF,
        DESCRIPTIVE,
        DEBUG,
        MAX_VERBOSITY_LEVEL,
        INHERIT
    } verbosity_level_t;
    verbosity_level_t level;

    static inline std::string verbosity_level_to_string(verbosity_level_t verb_level)
    {
        switch(verb_level)
        {
            case SILENT: return "SILENT"; break;
            case LOG: return "LOG"; break;
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
    assertion_level_t assert_level;

    static inline std::string assert_level_to_string(assertion_level_t assert_level)
    {
        switch(assert_level)
        {
            case ASSERT_NEVER: return "ASSERT_NEVER"; break;
            case ASSERT_CRITICAL: return "ASSERT_CRITICAL"; break;
            case ASSERT_CASUAL: return "ASSERT_CASUAL"; break;
        }
    }

    inline verbosity(verbosity_level_t verb_level = LOG, std::ostream& targetStream = std::cout)
    {
        _outputstream = &targetStream;
        level = verb_level;
        _streamtype = verbosity::LOG;
        assert_level = ASSERT_CRITICAL;
        buffer.clear();
    }

    /*!
     * \brief Get a reference to the verbosity object's stream
     * \return The current stream of the verbosity object
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

    inline verbosity& operator<<(const verbosity& verb_right)
    {
        return *this;
    }

    inline bool assert(bool condition, assertion_level_t importance, std::string brief_explanation,
                       std::string desc_explanation = "")
    {
        if(!condition)
        {
            brief() << brief_explanation;
            desc() << desc_explanation;
            end();

            if( ASSERT_NEVER < importance && importance <= assert_level)
            {
                *_outputstream << "ASSERTION LEVEL: " << assert_level_to_string(importance);
                *_outputstream << " | MY ASSERTIVENESS LEVEL: "
                               << assert_level_to_string(assert_level) << std::endl;
                abort();
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


#endif // VERBOSITY_H
