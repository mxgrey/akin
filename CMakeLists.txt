cmake_minimum_required(VERSION 2.8.3)
project(akin)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected. Default to Release!")
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX /usr)
  MESSAGE(STATUS "Setting installation prefix to ${CMAKE_INSTALL_PREFIX} by default")
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

option(build_Core_Only "Build only the core akin library" OFF)
option(build_osgAkin "Build OpenSceneGraph support for akin" ON)
option(build_urdfAkin "Build URDF support for akin" ON)
option(build_HuboKin "Build the Hubo extension of akin" ON)

if(build_Core_Only)
  set(build_osgAkin OFF)
  set(build_urdfAkin OFF)
  set(build_HuboKin OFF)
endif(build_Core_Only)

set(COMPLETED_BUILD TRUE)

add_definitions("-Wall -Wextra")

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}" )


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
set(HEADER_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/utils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(akin)

add_subdirectory(utils)

if(build_HuboKin)
  if(build_urdfAkin)
    MESSAGE(STATUS "Building HuboKin")
    add_subdirectory(HuboKin)
  else(build_urdfAkin)
    MESSAGE(WARNING "Could NOT build HuboKin, because it depends on urdfAkin!")
  endif(build_urdfAkin)
else(build_HuboKin)
  MESSAGE(STATUS "Ignoring HuboKin")
endif(build_HuboKin)

if(COMPLETED_BUILD)

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

else(COMPLETED_BUILD)

    message(STATUS "The entire package was NOT compiled, so we will not generate apps and tests!\n")

endif(COMPLETED_BUILD)
