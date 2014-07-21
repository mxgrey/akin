cmake_minimum_required(VERSION 2.6)

get_filename_component(dir_name ${CMAKE_CURRENT_LIST_DIR} NAME)

include_directories(${PROJECT_SOURCE_DIR})

file(GLOB dir_source "src/*.cpp" "src/*.c" "*.hpp" "*.h")
list(SORT dir_source)
add_library(${dir_name} ${dir_source})
install(TARGETS ${dir_name} DESTINATION ${LIBRARY_INSTALL_PATH})

file(GLOB include_files "*.h*")
LIST(SORT include_files)
install(FILES ${include_files} DESTINATION "${HEADER_INSTALL_PATH}/${dir_name}")
