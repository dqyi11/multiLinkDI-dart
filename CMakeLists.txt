cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

project(multiLinkDI)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

find_package(DART 6.2.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

find_package(JsonCpp REQUIRED)
if(NOT JSONCPP_FOUND)
   message("Jsoncpp not found")
endif()
include_directories(${JSONCPP_INCLUDE_DIR})
link_libraries(${JSONCPP_LIBRARY})

find_program(GENGETOPT gengetopt)
if( NOT GENGETOPT )
  message( FATAL_ERROR "gengetopt not found\n" )
  return()
endif()

macro(generate_ggo input output)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${output}.c ${CMAKE_CURRENT_BINARY_DIR}/${output}.h
    COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/${input} --file-name ${output} --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${input}
  )
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
endmacro(generate_ggo)

include_directories(${DART_INCLUDE_DIRS}
                    include/)

FILE(GLOB_RECURSE LibFiles "include/*.h"
                           "include/*.hpp")
add_custom_target(headers SOURCES ${LibFiles})

generate_ggo(src/load_problem.ggo load_problem_cmdline)

add_executable(load_problem src/load_problem.cpp
                            ${CMAKE_CURRENT_BINARY_DIR}/load_problem_cmdline.c)
target_link_libraries(load_problem ${DART_LIBRARIES}
                                   ${PROJECT_NAME}
                                   ${JSONCPP_LIBRARY})

add_library(${PROJECT_NAME} src/MultiLinkDI.cpp
                            src/MultiLinkDIWindow.cpp)
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})

add_executable(${PROJECT_NAME}_test test/test_multilinkdi.cpp)
target_link_libraries(${PROJECT_NAME}_test ${DART_LIBRARIES}
                                           ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_threelinks_test test/test_threelinks.cpp)
target_link_libraries(${PROJECT_NAME}_threelinks_test ${DART_LIBRARIES}
                                                      ${PROJECT_NAME})



