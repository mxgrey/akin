cmake_minimum_required(VERSION 2.8.3)

find_package(OpenSceneGraph 3.0 QUIET COMPONENTS osg osgViewer osgManipulator osgGA osgDB osgUtil)
set(osg_FOUND ${OSG_LIBRARY})
if(osg_FOUND)

  include_directories(${OpenSceneGraph_INCLUDE_DIRS})
  add_definitions(${OpenSceneGraph_DEFINITIONS})

  set(OSG_LIBRARIES     ${OSG_LIBRARY}
                        ${OSGVIEWER_LIBRARY}
                        ${OSGDB_LIBRARY}
                        ${OSGMANIPULATOR_LIBRARY}
                        ${OSGGA_LIBRARY}
                        ${OSGUTIL_LIBRARY}
           CACHE STRING "Libraries for OpenSceneGraph")

  get_filename_component(dir_name ${CMAKE_CURRENT_LIST_DIR} NAME)

  file(GLOB dir_source "src/*.cpp" "src/*.c" "*.hpp" "*.h")
  list(SORT dir_source)

  add_library(${dir_name} ${dir_source})
  target_link_libraries(${dir_name} ${OSG_LIBRARIES})

  install(TARGETS ${dir_name} DESTINATION ${LIBRARY_INSTALL_PATH})

  file(GLOB include_files "*.h*")
  LIST(SORT include_files)
  install(FILES ${include_files} DESTINATION "${HEADER_INSTALL_PATH}/${dir_name}")
  
else(osg_FOUND)
  MESSAGE(WARNING "Could NOT find OpenSceneGraph -- We will NOT build osgAkin!")
  set(COMPLETED_BUILD FALSE CACHE BOOL INTERNAL)
  set(didBuild_osgAkin FALSE CACHE BOOL INTERNAL)
endif(osg_FOUND)
