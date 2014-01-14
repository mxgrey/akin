
#include "GraphicsWindow.h"

using namespace akin;

GraphicsWindow* GraphicsWindow::_window = 0;

void GraphicsWindow::run() { glutMainLoop(); }

void GraphicsWindow::_keyboard(unsigned char key, int x, int y)
{
    switch(key)
    {
        case 'T':
        case 't':
        {
            break;
        }
    case 'w':
    case 'W':
    {
        simple_move_camera(0, 0, -0.1);
        break;
    }
    case 's':
    case 'S':
    {
        simple_move_camera(0, 0, 0.1);
        break;
    }
    case 'a':
    case 'A':
    {
        simple_move_camera(-0.1, 0, 0);
        break;
    }
    case 'd':
    case 'D':
    {
        simple_move_camera(0.1, 0, 0);
        break;
    }
    case 'f':
    case 'F':
    {
        simple_move_camera(0, 0.1, 0);
        break;
    }
    case 'v':
    case 'V':
    {
        simple_move_camera(0, -0.1, 0);
    }
    }
}

void GraphicsWindow::simple_move_camera(float dx, float dy, float dz)
{
    // THIS CODE IS GARBAGE AND SHOULD BE REPLACED PROMPTLY
    
    // 0  4  8 12
    // 1  5  9 13
    // 2  6 10 14
    // 3  7 11 15
    
    GraphicsBuffer::_buffer->_ViewMatrix.m[12]  -= dx;
    GraphicsBuffer::_buffer->_ViewMatrix.m[13]  -= dy;
    GraphicsBuffer::_buffer->_ViewMatrix.m[14]  -= dz;
}

void GraphicsWindow::static_keyboard(unsigned char key, int x, int y)
{
    _window->_keyboard(key, x, y);
}

void GraphicsWindow::Resize(int new_width, int new_height)
{
    _window->_current_width = new_width;
    _window->_current_height = new_height;

    glViewport(0, 0, new_width, new_height);
    _window->verb.debug() << "Set the viewport check"; _window->verb.end();
    CheckGLError(_window->verb, "Set the viewport");

    GraphicsBuffer::setProjectionMatrix(60, (float)(new_width)/(float)(new_height), 1.0f, 100.0f,
                                        true);
    CheckGLError(_window->verb, "Set the projection matrix");
    _window->verb.debug() << "Viewport set"; _window->verb.end();
}

void GraphicsWindow::Render()
{
//    CheckGLError(_window->verb, "About to clear screen");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//    CheckGLError(_window->verb, "Could not clear screen");

    GraphicsBuffer::drawElements();

    glutSwapBuffers();
    glutPostRedisplay();
}

GraphicsWindow::GraphicsWindow(int argc, char *argv[], std::string name, int width, int height,
                               verbosity::verbosity_level_t report_level)
{
    verb.level = report_level;
    if(verb.level == verbosity::INHERIT)
        verb.level = verbosity::LOG;

    makeInstance(argc, argv, name, width, height);
    if(checkInstance())
        _window->verb.level = verb.level;
}

GraphicsWindow::GraphicsWindow(bool create)
{

}

void GraphicsWindow::makeInstance(int argc, char *argv[], std::string name, int init_width, int init_height)
{
    if(_window == 0)
    {
        verb.debug() << "No OpenGL window created yet. Instantiating a new one!"; verb.end();
        _window = new GraphicsWindow(true);
        _window->_current_width = init_width;
        _window->_current_height = init_height;
        _window->_window_name = name;
        verb.debug() << "Finished instantiation"; verb.end();

        _Initialize(argc, argv, name, init_width, init_height);

        GraphicsBuffer instantiation(verb.level);
        // ^ This makes sure that the graphics buffer has been properly created at start-up
    }
}

bool GraphicsWindow::checkInstance()
{
    if(_window == 0)
    {
        verb.debug() << "No OpenGL window created yet!"; verb.end();
        return false;
    }

    return true;
}

void GraphicsWindow::_Initialize(int argc, char *argv[], std::string name, int init_width, int init_height)
{
    verb.debug() << "Initializing GLUT"; verb.end();
    glutInit(&argc, argv);

    verb.debug() << "Initializing GLUT context"; verb.end();
    glutInitContextVersion(4, 0);
    glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
    glutInitContextProfile(GLUT_CORE_PROFILE);

    verb.debug() << "Setting GLUT options"; verb.end();
    glutSetOption( GLUT_ACTION_ON_WINDOW_CLOSE,
                   GLUT_ACTION_GLUTMAINLOOP_RETURNS);

    verb.debug() << "Setting GLUT window size"; verb.end();
    glutInitWindowSize(init_width, init_height);

    verb.debug() << "Setting GLUT display mode"; verb.end();
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

    verb.debug() << "Creating GLUT window"; verb.end();
    _window_handle = glutCreateWindow(name.c_str());

    verb.Assert(_window_handle >= 1, verbosity::ASSERT_CRITICAL,
                "Could not create a new rendering window!");

    verb.debug() << "Assigning callback functions"; verb.end();
    glutReshapeFunc(akin::GraphicsWindow::Resize);
    glutDisplayFunc(akin::GraphicsWindow::Render);
    glutCloseFunc(akin::GraphicsBuffer::Cleanup);
    glutKeyboardFunc(akin::GraphicsWindow::static_keyboard);

    CheckGLError(verb, "Checking errors before initializing GLEW");

    // TODO: Why does glewExperimental need to be true?? Answer: dumb GLEW bug
    glewExperimental = GL_TRUE;
    CheckGLError(verb, "About to init GLEW");
    GLenum GlewInitResult = glewInit();
    verb.Assert(GLEW_OK == GlewInitResult, verbosity::ASSERT_CRITICAL,
                "Could not initialize GLEW",
                ": "+std::string((char*)glewGetErrorString(GlewInitResult)));
    glGetError();

    CheckGLError(verb, "About to set the clear color");
    glClearColor(0.85f, 0.85f, 0.85f, 0.0f);

    glEnable(GL_DEPTH_TEST);
    CheckGLError(verb, "Could not enable depth testing options");
    glDepthFunc(GL_LESS);
    CheckGLError(verb, "Could not set OpenGL depth testing options");

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    CheckGLError(verb, "Could not set OpenGL culling options");


    verb.log() << "INFO: OpenGL Version: "+std::string((char*)glGetString(GL_VERSION)); verb.end();
}
