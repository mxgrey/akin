#ifndef GRAPHICSBUFFER_H
#define GRAPHICSBUFFER_H

#include "IncludeGL.h"
#include "GraphicsObject.h"
#include "map"
#include "GraphicsShader.h"

namespace akin {


typedef struct
{
    float m[16];
} FloatMatrix;

inline FloatMatrix convertToFloat(const Eigen::Isometry3d& other)
{
    FloatMatrix M;
    for(size_t i=0; i<4; ++i)
        for(size_t j=0; j<4; ++j)
            M.m[4*i+j] = other(j,i);

    return M;
}

inline FloatMatrix FloatZeros()
{
    FloatMatrix M;
    memset(M.m, sizeof(M.m), 0);
    return M;
}

inline FloatMatrix FloatIdentity()
{
    return convertToFloat(Eigen::Isometry3d::Identity());
}

typedef std::vector<GLuint> IdArray;
typedef std::map<uint,GraphicsObject*> GraphicsPointerMap;

typedef struct {
    float XYZW[4];
    float RGBA[4];
} TestVertex;



class GraphicsBuffer
{
public:

    GraphicsBuffer(verbosity::verbosity_level_t report_level = verbosity::LOG);

    static void Cleanup();
    static void CreateVBO();
    static void DestroyVBO();
    static void CreateShaders();
    static void DestroyShaders();
    
    static void drawElements();

    /*!
     * \fn displayGraphic();
     * \brief Add the graphic to the list of things to be rendered
     *
     * This will register your GraphicsObject as something which should
     * be rendered in the display. However, if your object is destroyed
     * (for example, if it falls out of scope during runtime), then it
     * can no longer be displayed. To make sure that this does not happen,
     * you can use storeGraphic().
     */
    static void displayGraphic(GraphicsObject& object);

    /*!
     * \fn unstageGraphic();
     * \brief Remove the graphic from the list of things to be rendered
     *
     * This will take your graphic out of the queue of graphics to be
     * rendered in the display.
     */
    static void unstageGraphic(GraphicsObject& object);

    /*!
     * \fn storeGraphic()
     * \brief Store the graphic in persistent memory
     * \return Unsigned int which provides a unique identifier for your stored graphic
     *
     * This function will ensure that your graphic continues to exist
     * for as long as your program is running, or until you run the
     * function deleteGrahpic().
     *
     * The return value will start from 0 and increase by 1 throughout execution, even
     * if earlier graphics get deleted. If the ID of a deleted graphic is passed in
     * to any function, that function will either not perform an operation, or it could
     * print out an error and even abort, depending on verbosity and assertiveness
     * settings.
     *
     * displayGraphic() is run automatically at the end of this function
     */
    static uint storeGraphic(const GraphicsObject& object);

    /*!
     * \fn retrieveGraphic()
     * \brief Provides a reference to a stored GraphicsObject
     * \param graphicId
     * \return Reference to a stored GraphicsObject corresponding to graphicId
     *
     * If this attempts to access a graphic which has been deleted, it will
     * return an empty GraphicsObject.
     */
    static GraphicsObject& retrieveGraphic(uint graphicId);

    /*!
     * \fn deleteGrahpic()
     * \brief Removes the designated graphic from storage, freeing up its memory
     * \param graphicId
     *
     * This deletes the persistent graphic created by an earlier call to storeGraphic().
     * The graphicId will no longer point to any graphic.
     */
    static void deleteGrahpic(uint graphicId);

    /*!
     * \fn displayStoredGraphic();
     * \brief Equivalent to displayGraphic( retrieveGraphic( graphicId ) );
     * \param graphicId
     */
    static void displayStoredGraphic(uint graphicId);

    /*!
     * \fn unstageStoredGraphic();
     * \brief Equivalent to unstageGraphic( retrieveGraphic( graphicId ) );
     * \param graphicId
     */
    static void unstageStoredGraphic(uint graphicId);


    static void setProjectionMatrix(float field_of_view,
                                    float aspect_ratio,
                                    float near_plane,
                                    float far_plane);

    verbosity verb;

protected:
    
    GLuint _testVertexAddress;
    GLuint _testFaceAddress;
    GLuint _testSize;
    void _createTestObject();

    GraphicsPointerArray _graphics;
    GraphicsShader _vertexShader;
    GraphicsShader _fragmentShader;
    GLuint _shaderProgramId;

    GraphicsPointerMap _storedGraphics;
    uint _nextStorageIndex;

    GraphicsObject _emptyGraphic;

    FloatMatrix _ProjectionMatrix;
    FloatMatrix _ViewMatrix;
    FloatMatrix _ModelMatrix;

    GLint _ProjectionMatrixId;
    GLint _ViewMatrixId;
    GLint _ModelMatrixId;

    static GraphicsBuffer* _buffer;

    void _makeBuffer(verbosity::verbosity_level_t report_level);

private:

    GraphicsBuffer(bool create, verbosity::verbosity_level_t report_level);

};


} // namespace akin


#endif // GRAPHICSBUFFER_H
