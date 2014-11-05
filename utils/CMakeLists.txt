cmake_minimum_required(VERSION 2.8.3)

# TODO: I could easily generalize this to iterate through the directories

if(build_urdfAkin)
  MESSAGE(STATUS "Building urdfAkin")
  set(didBuild_urdfAkin TRUE CACHE BOOL INTERNAL)
  add_subdirectory(urdfAkin)
else(build_urdfAkin)
  MESSAGE(STATUS "Ignoring urdfAkin")
  set(COMPLETED_BUILD FALSE CACHE BOOL INTERNAL)
  set(didBuild_urdfAkin FALSE CACHE BOOL INTERNAL)
endif(build_urdfAkin)


if(build_osgAkin)
  MESSAGE(STATUS "Building osgAkin")
  set(didBuild_osgAkin TRUE CACHE BOOL INTERNAL)
  add_subdirectory(osgAkin)
else(build_osgAkin)
  MESSAGE(STATUS "Ignoring osgAkin")
  set(COMPLETED_BUILD FALSE CACHE BOOL INTERNAL)
  set(didBuild_osgAkin FALSE CACHE BOOL INTERNAL)
endif(build_osgAkin)


if(build_dartAkin)
  MESSAGE(STATUS "Building dartAkin")
  set(didBuild_dartAkin TRUE CACHE BOOL INTERNAL)
  add_subdirectory(dartAkin)
else(build_dartAkin)
  MESSAGE(STATUS "Ignoring dartAkin")
  set(didBuild_dartAkin FALSE CACHE BOOL INTERNAL)
endif(build_dartAkin)
