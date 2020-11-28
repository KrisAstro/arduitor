PROJECT(arduitor_rotator CXX C)
cmake_minimum_required(VERSION 3.0)

set (ARDUITOR_VERSION_MAJOR 0)
set (ARDUITOR_VERSION_MINOR 1)

include(GNUInstallDirs)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set(BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(INDI REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_arduitor.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_arduitor_rotator.xml)

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})


################ ARDUITOR ################

set(ARDUITOR_rotator_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/arduitor.cpp)

add_executable(indi_arduitor_rotator ${ARDUITOR_rotator_SRCS})
target_link_libraries(indi_arduitor_rotator indidriver)
install(TARGETS indi_arduitor_rotator RUNTIME DESTINATION bin)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_arduino_rotator.xml DESTINATION ${INDI_DATA_DIR})
