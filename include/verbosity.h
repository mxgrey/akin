#ifndef VERBOSITY_H
#define VERBOSITY_H

#include <string>
#include <iostream>


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
        DEBUG
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
        }
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
        ASSERT_PERMISSIVE=0,
        ASSERT_CRITICAL,
        ASSERT_STRICT
    } assertion_level_t;
    assertion_level_t assert_level;

    inline verbosity(std::ostream& targetStream = std::cout)
    {
        _outputstream = &targetStream;
        level = verbosity::LOG;
        assert_level = ASSERT_CRITICAL;
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
            *_outputstream << message;
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
        *_outputstream << /*"\n" <<*/ std::endl;
        _streamtype = LOG; // TODO: Consider making this SILENT
        return *this;
    }

    inline verbosity& operator<<(const verbosity& verb_right)
    {
        return *this;
    }

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

} // namespace akin


#endif // VERBOSITY_H
