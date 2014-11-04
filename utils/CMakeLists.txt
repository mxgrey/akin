cmake_minimum_required(VERSION 2.8.3)

if(build_urdfAkin)
  MESSAGE(STATUS "Building urdfAkin")
  add_subdirectory(urdfAkin)
else(build_urdfAkin)
  MESSAGE(STATUS "Ignoring urdfAkin")
  set(COMPLETED_BUILD FALSE CACHE INTERNAL)
endif(build_urdfAkin)


if(build_osgAkin)
  MESSAGE(STATUS "Building osgAkin")
  add_subdirectory(osgAkin)
else(build_osgAkin)
  MESSAGE(STATUS "Ignoring osgAkin")
  set(COMPLETED_BUILD FALSE CACHE INTERNAL)
endif(build_osgAkin)


if(build_dartAkin)
  MESSAGE(STATUS "Building dartAkin")
  add_subdirectory(dartAkin)
else(build_dartAkin)
  MESSAGE(STATUS "Ignoring dartAkin")
endif(build_dartAkin)
