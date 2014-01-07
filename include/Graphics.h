#ifndef GRAPHICS_H
#define GRAPHICS_H

#include "verbosity.h"

extern "C" {
#include <stdlib.h>
#include <string.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
}

namespace akin {

class GraphicsWindow
{
public:
    
    GraphicsWindow(int width=800, int height=600, std::string name = "window");
    
    void Initialize(int argc, char* argv[]);
    
    void Resize(int new_width, int new_height);
    void Render();
    
    verbosity verb;
    
protected:
    
    void InitWindow(int argc, char* argv[]);
    
    int current_width;
    int current_height;
    int window_handle;
    std::string window_name;
    
};

} // namespace akin

#endif // GRAPHICS_H
