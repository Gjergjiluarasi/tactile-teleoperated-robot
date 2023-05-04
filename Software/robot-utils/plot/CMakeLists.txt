cmake_minimum_required(VERSION 3.4)

project(plot)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

find_package(Eigen3)

if (${EIGEN3_FOUND})
  include_directories(${EIGEN3_INCLUDE_DIR})
else()
  message(STATUS "Eigen3 not found")
endif()

find_package(Python3 COMPONENTS Interpreter Development)
if (${Python3_FOUND})
  include_directories(${Python3_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Python3 not found, please install it.")
endif()

find_package(NumPy)
if (${PYTHON_NUMPY_FOUND})
  include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
else()
  message(WARNING "Python3 NumPy not found, proceeding with -DWITHOUT_NUMPY."
  " Some functions might not work.")
  add_definitions(-DWITHOUT_NUMPY)
endif()

# add_subdirectory(examples)

# set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

# add WITH_EIGEN, if specified
if(WITH_EIGEN)
  add_definitions("-DWITH_EIGEN")
endif()

# add the exectuable and link it to the Python libs
add_executable(basic basic.cpp ../matplotlibcpp.h)
target_link_libraries(basic ${Python3_LIBRARIES})