cmake_minimum_required(VERSION 3.16.0)

project(6G-life CXX)

message(STATUS "project source dir: \t\t${PROJECT_SOURCE_DIR}")
message(STATUS "cmake source dir: \t\t${CMAKE_SOURCE_DIR}")
message(STATUS "cmake current list dir: \t${CMAKE_CURRENT_LIST_DIR}")

# list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")
message(STATUS "cmake module dir: \t\t${CMAKE_MODULE_PATH}")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Used only for DEBUG build version
set(CMAKE_DEBUG_POSTFIX d) 
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")

# Used only for RELEASE build version
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")

include(CTest)
enable_testing()

set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
message(STATUS "cmake include dir: \t\t${INCLUDE_DIR}")
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
message(STATUS "cmake custom source dir: \t\t${SOURCE_DIR}")

###################### EndoWrist part ######################

# Set libEthercat path variable
set(LIBETHERCAT_DIR /usr/local/etherlab/lib)
message(STATUS "libEthercat dir: \t\t${LIBETHERCAT_DIR}")

# Set libEposCmd path variable
set(LIBEPOSCMD_DIR /usr/lib/)
message(STATUS "libEposCmd dir: \t\t\t${LIBEPOSCMD_DIR}")

# Find libEposCmd 
find_library(LIBEPOSCMD NAMES libEposCmd.so PATHS  ${LIBEPOSCMD_DIR})
if ( NOT LIBEPOSCMD )
  message( WARNING "libEposCmd library NOT FOUND  in ${LIBEPOSCMD_DIR}- the respective targets won't be build")
else()
  message( STATUS "libEposCmd library : \t${LIBEPOSCMD}")
endif( NOT LIBEPOSCMD )

# Active build using shared libraries
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# Add libEthercat and libEposCmd link directories
link_directories(${LIBETHERCAT_DIR} ${LIBEPOSCMD_DIR})

# Add GLOBAL if the imported library is located in directories above the current.
add_library(libEposCmd SHARED IMPORTED GLBall)
# You can define two import-locations: one for debug and one for release.
set_target_properties(libEposCmd PROPERTIES IMPORTED_LOCATION ${LIBEPOSCMD_DIR}/libEposCmd.so)

# Add robotUtils custom library
add_library(robotUtils SHARED 
            ${INCLUDE_DIR}/configUtils.cpp
            ${INCLUDE_DIR}/udpUtils.cpp
            ${INCLUDE_DIR}/robot.cpp
            ${INCLUDE_DIR}/frankaEmika.cpp
            ${INCLUDE_DIR}/endoWrist.cpp
            ${INCLUDE_DIR}/eposMotorController.cpp
            ${INCLUDE_DIR}/trajectory.cpp
            ${INCLUDE_DIR}/pugixml.cpp)

###################### Franka Emika part ######################

if(NOT FRANKA_IS_FOUND)
    find_package(Franka REQUIRED)
endif()

# find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Add examples_common library
add_library(examples_common STATIC ${INCLUDE_DIR}/examples_common.cpp)

# Add rcmKinematics library
# add_library(rcmKinematics STATIC ${INCLUDE_DIR}/rcmKinematics.cpp)

# Add PandaKinematics_multi library
add_library(PandaKinematics_multi STATIC ${INCLUDE_DIR}/PandaKinematics_multi.cpp)
#add_library(PandaKinematics_multi_11DoFs STATIC ${INCLUDE_DIR}/PandaKinematics_multi_11DoFs.cpp)
#add_library(PandaKinematics_withoutRCMandendo STATIC ${INCLUDE_DIR}/PandaKinematics_withoutRCMandendo.cpp)

# Add Recorder library
add_library(Recorder STATIC ${INCLUDE_DIR}/Recorder.cpp)

###################### Utilities part ######################
find_package(Eigen3)

if (${EIGEN3_FOUND})
  message(STATUS "Eigen3 found")
  include_directories(${EIGEN3_INCLUDE_DIR})
else()
  message(STATUS "Eigen3 not found")
endif()

find_package(Python3 COMPONENTS Interpreter Development)
if (${Python3_FOUND})
  message(STATUS "Python3 found")
  include_directories(${Python3_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Python3 not found, please install it.")
endif()

find_package(NumPy)
if (${PYTHON_NUMPY_FOUND})
  message(STATUS "NumPy found")
  include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
else()
  message(WARNING "Python3 NumPy not found, proceeding with -DWITHOUT_NUMPY."
  " Some functions might not work.")
  add_definitions(-DWITHOUT_NUMPY)
endif()

###################### Executables part ######################

set(EXECUTABLES ${PROJECT_NAME})

foreach(executable ${EXECUTABLES})
    add_executable(${executable} ${SOURCE_DIR}/${executable}.cpp)
    target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIR} PRIVATE ${SOURCE_DIR})
    target_link_libraries(${executable} 
                            Franka::Franka
                            examples_common
                            # rcmKinematics
                            robotUtils
                            libEposCmd
                            ethercat
                            PandaKinematics_multi
                            #PandaKinematics_multi_11DoFs
                            #PandaKinematics_withoutRCMandendo
                            Recorder
                            ${Python3_LIBRARIES} 
                            Threads::Threads 
                            Poco::Foundation)
                            # Eigen3::Eigen3)
endforeach()

include(GNUInstallDirs)
install(TARGETS ${EXECUTABLES}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})