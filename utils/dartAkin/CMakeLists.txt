cmake_minimum_required(VERSION 2.8.3)

find_package(DART)
if(DART_FOUND)

  include_directories(${DART_INCLUDE_DIRS})

  get_filename_component(dir_name ${CMAKE_CURRENT_LIST_DIR} NAME)

  file(GLOB dir_source "src/*.cpp" "src/*.c" "*.hpp" "*.h")
  list(SORT dir_source)

  add_library(${dir_name} ${dir_source})
  install(TARGETS ${dir_name} DESTINATION ${LIBRARY_INSTALL_PATH})

  file(GLOB include_files "*.h")
  LIST(SORT include_files)
  install(FILES ${include_files} DESTINATION "${HEADER_INSTALL_PATH}/${dir_name}")

  # Unit Testing
  set(exec_dependencies akin dartAkin)
  enable_testing()

  file(GLOB unit_tests_source "test/*.cpp")
  LIST(SORT unit_tests_source)

  message(STATUS "\n    ${dir_name} Unit Tests: ")
  add_custom_target(${dir_name}_check COMMAND ${CMAKE_CTEST_COMMAND})
  foreach(utest_src_file ${unit_tests_source})
    get_filename_component(test_base ${utest_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}")
    add_executable(${test_base} ${utest_src_file})
    target_link_libraries(${test_base} ${exec_dependencies})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(${dir_name}_check ${test_base})
  endforeach(utest_src_file)

else(DART_FOUND)
  MESSAGE(WARNING "Could NOT find DART -- We will NOT build dartAkin!")
  # TODO: Decide whether COMPLETED_BUILD is needed here... hopefully not
  set(didBuild_dartAkin FALSE CACHE BOOL INTERNAL)
endif(DART_FOUND)
