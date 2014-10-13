cmake_minimum_required(VERSION 2.8.3)

if(build_urdfAkin)
  MESSAGE(STATUS "Building urdfAkin")
  add_subdirectory(urdfAkin)
else(build_urdfAkin)
  MESSAGE(STATUS "Ignoring urdfAkin")
  set(COMPLETED_BUILD FALSE PARENT_SCOPE)
endif(build_urdfAkin)


if(build_osgAkin)
  MESSAGE(STATUS "Building osgAkin")
  add_subdirectory(osgAkin)
else(build_osgAkin)
  MESSAGE(STATUS "Ignoring osgAkin")
  set(COMPLETED_BUILD FALSE PARENT_SCOPE)
endif(build_osgAkin)


set(COMPLETED_BUILD ${COMPLETED_BUILD} PARENT_SCOPE)
