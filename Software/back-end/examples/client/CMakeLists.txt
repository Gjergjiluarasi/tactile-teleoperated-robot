cmake_minimum_required(VERSION 3.1.13)
project(client VERSION 0.1.0)

include(CTest)
enable_testing()

include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/../../include PRIVATE ${CMAKE_SOURCE_DIR})
set(SOURCES main.cpp client.cpp ${CMAKE_SOURCE_DIR}/../../include/udpUtils.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

#install(TARGETS server DESTINATION bin)