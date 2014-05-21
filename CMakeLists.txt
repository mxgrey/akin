cmake_minimum_required(VERSION 2.6)
project(akin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
set(HEADER_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)

include_directories(${PROJECT_SOURCE_DIR})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## TODO: Seperate the graphics out into their own library
file(GLOB lib_source "src/*.cpp" "akin/*.h")
list(SORT lib_source)

file(GLOB unit_tests_source "test/*.cpp")
LIST(SORT unit_tests_source)

add_library(${PROJECT_NAME} SHARED ${lib_source})


find_package(OpenSceneGraph)

file(GLOB osg_source "osg/*.cpp" "osgAkin/*.h")
LIST(SORT osg_source)
add_library(osgAkin SHARED ${osg_source})


include_directories(${OpenSceneGraph_INCLUDE_DIRS})
add_definitions(${OpenSourceGraph_DEFINITIONS})
link_libraries(osg osgViewer osgDB osgAkin)

option(URDF_PARSING "include urdf parsing" ON)

if(URDF_PARSING)

  find_package(urdfdom QUIET)
  if(urdfdom_FOUND)
    find_package( urdfdom_headers QUIET )
    if(urdfdom_headers_FOUND)
      include_directories( ${urdfdom_headers_INCLUDE_DIRS} )
      include_directories( ${urdfdom_INCLUDE_DIRS} )

      MESSAGE(STATUS "Found urdfdom -- We will compile a URDF parser")
      file(GLOB urdf_source "urdf/*.cpp" "akinUtils/*.h")
      list(SORT urdf_source)

      link_libraries(${urdfdom_LIBRARIES})

      add_library(akinUtils SHARED ${urdf_source})
      link_libraries(akinUtils)
      target_link_libraries(${PROJECT_NAME} akinUtils)

      set_property(TARGET ${PROJECT_NAME} PROPERTY COMPILE_DEFINITIONS "HAVE_URDFPARSING")

      set( AKINUTILS_VERIFIED "true" )

    else(urdfdom_headers_FOUND)
      MESSAGE(STATUS "Could NOT find urdfdom *headers* -- We will NOT compile a URDF parser")
    endif(urdfdom_headers_FOUND)
  else( urdfdom_FOUND)
    MESSAGE(STATUS "Could NOT find urdfdom -- We will NOT compile a URDF parser")
  endif(urdfdom_FOUND)

endif(URDF_PARSING)


enable_testing()

message(STATUS "\n -- UNIT TEST: ")
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
foreach(utest_src_file ${unit_tests_source})
    get_filename_component(test_base ${utest_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}")
    add_executable(${test_base} ${utest_src_file})
    target_link_libraries(${test_base} ${PROJECT_NAME} ${OpenSceneGraph_LIBRARIES})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check ${test_base})
endforeach(utest_src_file)

file(GLOB akin_includes "akin/*.h")
LIST(SORT akin_includes)
install(FILES ${akin_includes} DESTINATION "${HEADER_INSTALL_PATH}/akin")

file(GLOB akinUtils_includes "akinUtils/*.h")
LIST(SORT akinUtils_includes)
install(FILES ${akinUtils_includes} DESTINATION "${HEADER_INSTALL_PATH}/akinUtils")

file(GLOB osgAkin_includes "osgAkin/*.h")
LIST(SORT osgAkin_includes "osgAkin/*.h")
install(FILES ${osgAkin_includes} DESTINATION "${HEADER_INSTALL_PATH}/osgAkin")

install(TARGETS ${PROJECT_NAME} osgAkin DESTINATION ${LIBRARY_INSTALL_PATH})

if(AKINUTILS_VERIFIED)
    install(TARGETS ${akinUtils} DESTINATION ${LIBRARY_INSTALL_PATH})
endif(AKINUTILS_VERIFIED)







