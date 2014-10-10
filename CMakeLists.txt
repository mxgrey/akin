cmake_minimum_required(VERSION 2.6)
project(akin)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected. Default to Release!")
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

add_definitions("-Wall -Wextra")

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}" )

set(COMPILE_EVERYTHING ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
set(HEADER_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)

include_directories(${PROJECT_SOURCE_DIR})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(akin)

find_package(OpenSceneGraph)
add_subdirectory(osgAkin)

include_directories(${OpenSceneGraph_INCLUDE_DIRS})
add_definitions(${OpenSourceGraph_DEFINITIONS})

link_libraries(akin osg osgViewer osgDB osgGA osgUtil osgAkin)

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

      add_subdirectory(akinUtils)

      link_libraries(akinUtils)

    else(urdfdom_headers_FOUND)
      MESSAGE(STATUS "Could NOT find urdfdom *headers* -- We will NOT compile a URDF parser")
      set(COMPILE_EVERYTHING OFF)
    endif(urdfdom_headers_FOUND)
  else( urdfdom_FOUND)
    MESSAGE(STATUS "Could NOT find urdfdom -- We will NOT compile a URDF parser")
    set(COMPILE_EVERYTHING OFF)
  endif(urdfdom_FOUND)

endif(URDF_PARSING)

if(COMPILE_EVERYTHING)

    add_subdirectory(HuboKin)

    set(exec_dependencies ${PROJECT_NAME} ${OpenSceneGraph_LIBRARIES} HuboKin)

    enable_testing()

    file(GLOB unit_tests_source "test/*.cpp")
    LIST(SORT unit_tests_source)

    message(STATUS "\n    Unit Tests: ")
    add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
    foreach(utest_src_file ${unit_tests_source})
        get_filename_component(test_base ${utest_src_file} NAME_WE)
        message(STATUS "Adding test ${test_base}")
        add_executable(${test_base} ${utest_src_file})
        target_link_libraries(${test_base} ${exec_dependencies})
        add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
        add_custom_target(${test_base}.run ${test_base} ${ARGN})
        add_dependencies(check ${test_base})
    endforeach(utest_src_file)

    file(GLOB apps_source "apps/*.cpp")
    LIST(SORT apps_source)

    message(STATUS "\n    Applications: ")
    foreach(app_src_file ${apps_source})
        get_filename_component(app_base ${app_src_file} NAME_WE)
        message(STATUS "Adding application ${app_base}")
        add_executable(${app_base} ${app_src_file})
        target_link_libraries(${app_base} ${exec_dependencies})
    endforeach(app_src_file)

else(COMPILE_EVERYTHING)

    message(STATUS "\n The entire package was not compiled, so we will not generate apps and tests!")

endif(COMPILE_EVERYTHING)
