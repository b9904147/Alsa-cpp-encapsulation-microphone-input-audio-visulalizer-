cmake_minimum_required(VERSION 2.8.4)
project(ALSA_encapsulation)

set(EXEC_NAME ALSA_encapsulation)

find_package(ALSA) #alsa
find_package(Threads) #pthread

find_package(PkgConfig REQUIRED)
pkg_search_module(FFTW REQUIRED fftw3 IMPORTED_TARGET)
include_directories(PkgConfig::FFTW)
link_libraries     (PkgConfig::FFTW)

set(INCLUDE_DIR ./include)

include_directories(${INCLUDE_DIR} ${ALSA_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

set(SOURCE_FILES
  main.cc
  src/WavFunctions.cc
  src/AlsaControl.cc)

add_executable(${EXEC_NAME} ${SOURCE_FILES})

target_link_libraries(${EXEC_NAME} ${CMAKE_THREAD_LIBS_INIT} ${ALSA_LIBRARIES})
