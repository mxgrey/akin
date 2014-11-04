cmake_minimum_required(VERSION 2.8.3)

find_package(urdfdom QUIET)
if(urdfdom_FOUND)
  
  find_package(urdfdom_headers QUIET)
  if(urdfdom_headers_FOUND)

    get_filename_component(dir_name ${CMAKE_CURRENT_LIST_DIR} NAME)

    set(urdfdom_LIBRARIES ${urdfdom_LIBRARIES} CACHE STRING "Libraries for urdfdom")

    file(GLOB dir_source "src/*.cpp" "src/*.c" "*.hpp" "*.h")
    list(SORT dir_source)

    add_library(${dir_name} ${dir_source})

    install(TARGETS ${dir_name} DESTINATION ${LIBRARY_INSTALL_PATH})

    file(GLOB include_files "*.h*")
    LIST(SORT include_files)
    install(FILES ${include_files} DESTINATION "${HEADER_INSTALL_PATH}/${dir_name}")

  else(urdfdom_headers_FOUND)
    MESSAGE(WARNING "Could NOT find urdfdom *headers* -- We will NOT build urdfAkin")
    set(COMPLETED_BUILD FALSE CACHE INTERNAL)
    set(build_urdfAkin OFF CACHE BOOL FORCE)
  endif(urdfdom_headers_FOUND)

else(urdfdom_FOUND)
  MESSAGE(WARNING "Could NOT find urdfdom -- We will NOT build urdfAkin")
  set(COMPLETED_BUILD FALSE CACHE INTERNAL)
  set(build_urdfAkin OFF CACHE BOOL "Cannot build urdfAkin" FORCE)
endif(urdfdom_FOUND)
