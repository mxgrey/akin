
#include "Graphics.h"

using namespace akin;

GraphicsWindow::GraphicsWindow(int width, int height, std::string name) :
    current_width(width),
    current_height(height),
    window_name(name)
{
    
}

void GraphicsWindow::Initialize(int argc, char *argv[])
{
    InitWindow(argc, argv);
    
    verb.log() << "INFO: OpenGL Version: "+std::string((char*)glGetString(GL_VERSION));
    
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

void GraphicsWindow::InitWindow(int argc, char *argv[])
{
    glutInit(&argc, argv);
    
    glutInitContextVersion(4, 0);
    glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
    glutInitContextProfile(GLUT_CORE_PROFILE);
    
    glutSetOption( GLUT_ACTION_ON_WINDOW_CLOSE,
                   GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    
    glutInitWindowSize(current_width, current_height);
    
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    
    window_handle = glutCreateWindow(window_name.c_str());
    
    verb.Assert(window_handle >= 1, verbosity::ASSERT_CRITICAL,
                "Could not create a new rendering window!");
    
    glutReshapeFunc(&akin::GraphicsWindow::Resize));
}
