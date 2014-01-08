#ifndef INCLUDEGL_H
#define INCLUDEGL_H

#include "verbosity.h"

extern "C" {
#include <stdlib.h>
#include <string.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <stdio.h>
}

inline bool CheckGLError(akin::verbosity& verb, std::string message,
                         akin::verbosity::assertion_level_t assert_level = akin::verbosity::ASSERT_CRITICAL)
{
    GLenum GlErrorResult = glGetError();
    return verb.Assert(GlErrorResult==GL_NO_ERROR,
                       assert_level, message, ": "+std::string((char*)gluErrorString(GlErrorResult)));
}

#endif // INCLUDEGL_H
