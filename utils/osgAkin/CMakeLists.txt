cmake_minimum_required(VERSION 2.8.3)

find_package(OpenSceneGraph QUIET COMPONENTS osg osgViewer osgManipulator osgGA osgDB osgUtil)
if(OpenSceneGraph_FOUND)

  include_directories(${OpenSceneGraph_INCLUDE_DIRS})
  add_definitions(${OpenSourceGraph_DEFINITIONS})

  get_filename_component(dir_name ${CMAKE_CURRENT_LIST_DIR} NAME)

  file(GLOB dir_source "src/*.cpp" "src/*.c" "*.hpp" "*.h")
  list(SORT dir_source)

  add_library(${dir_name} ${dir_source})
  link_libraries(${dir_name})

  install(TARGETS ${dir_name} DESTINATION ${LIBRARY_INSTALL_PATH})

  file(GLOB include_files "*.h*")
  LIST(SORT include_files)
  install(FILES ${include_files} DESTINATION "${HEADER_INSTALL_PATH}/${dir_name}")
  
else(OpenSceneGraph_FOUND)
  MESSAGE(WARNING "Could NOT find OpenSceneGraph -- We will NOT build osgAkin!")
  set(COMPLETED_BUILD FALSE PARENT_SCOPE)
endif(OpenSceneGraph_FOUND)
