cmake_minimum_required(VERSION 2.8.3)

if(build_urdfAkin)
  MESSAGE(STATUS "Building urdfAkin")
  add_subdirectory(urdfAkin)
  set(build_urdfAkin ${build_urdfAkin} PARENT_SCOPE)
else(build_urdfAkin)
  MESSAGE(STATUS "Ignoring urdfAkin")
  set(COMPLETED_BUILD FALSE)
endif(build_urdfAkin)


if(build_osgAkin)
  MESSAGE(STATUS "Building osgAkin")
  add_subdirectory(osgAkin)
  set(build_osgAkin ${build_osgAkin} PARENT_SCOPE)
else(build_osgAkin)
  MESSAGE(STATUS "Ignoring osgAkin")
  set(COMPLETED_BUILD FALSE)
endif(build_osgAkin)


set(COMPLETED_BUILD ${COMPLETED_BUILD} PARENT_SCOPE)
