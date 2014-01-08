
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
            _buffer.setActiveIndexBuffer(
                        _buffer.getActiveIndexBuffer()==1 ? 0 : 1);
            break;
        }
    }
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
}

void GraphicsWindow::Render()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    GraphicsBuffer::drawElements();

    glutSwapBuffers();
    glutPostRedisplay();
}

GraphicsWindow::GraphicsWindow(int argc, char *argv[], std::string name, int width, int height,
                               verbosity::verbosity_level_t report_level) :
    _buffer(report_level)
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

    verb.debug() << "Initializing GLEW"; verb.end();

    // TODO: Why does glewExperimental need to be true??
    glewExperimental = GL_TRUE;
    GLenum GlewInitResult = glewInit();
    verb.Assert(GLEW_OK == GlewInitResult, verbosity::ASSERT_CRITICAL,
                "Could not initialize GLEW",
                ": "+std::string((char*)glewGetErrorString(GlewInitResult)));


    glClearColor(0.85f, 0.85f, 0.85f, 0.0f);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    CheckGLError(verb, "Could not set OpenGL depth testing options");

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    CheckGLError(verb, "Could not set OpenGL culling options");



    verb.debug() << "Creating shaders"; verb.end();
    _buffer.CreateShaders();
    verb.debug() << "Creating VBO"; verb.end();
    _buffer.CreateVBO();



    verb.log() << "INFO: OpenGL Version: "+std::string((char*)glGetString(GL_VERSION)); verb.end();
}
