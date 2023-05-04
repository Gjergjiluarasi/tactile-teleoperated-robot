cmake_minimum_required(VERSION 3.1.13)
project(server VERSION 0.1.0)

# Used only for DEBUG build version
set(CMAKE_DEBUG_POSTFIX d) 
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")

# Used only for RELEASE build version
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")
include(CTest)
enable_testing()

include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/../../include PRIVATE ${CMAKE_SOURCE_DIR})
set(SOURCES main.cpp server.cpp ${CMAKE_SOURCE_DIR}/../../include/udpUtils.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

#install(TARGETS server DESTINATION bin)