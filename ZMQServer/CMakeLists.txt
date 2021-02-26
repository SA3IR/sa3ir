# CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
cmake_minimum_required (VERSION 2.8)
project (ZMQServer)

#---- Enable C++14 ---- ////////////////
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest)

include_directories(
  include
  include/3rdparty
  include/queries
  include/client
  $ENV{ROQME_ROOT}/include
  $ENV{ROQME_ROOT}/cpp/roqmeDDS/idl
  $ENV{OSPL_HOME}/include/dcps/C++/isocpp2
  $ENV{OSPL_HOME}/include/sys
  $ENV{OSPL_HOME}/include
  $ENV{OSPL_HOME}/include/dcps/C++/SACPP
)

#add_definitions(-DSUBSCRIBE_TO_ROQME_CONTEXTS)
#add_definitions(-DSUBSCRIBE_TO_ENUM_CONTEXTS)
add_definitions(-DADAPTATION_LOGIC)

link_directories($ENV{OSPL_HOME}/lib)
link_directories($ENV{ROQME_ROOT}/cpp/roqmeDDS/lib)

file(GLOB_RECURSE SRC_${PROJECT_NAME} src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_${PROJECT_NAME}})
target_link_libraries(${PROJECT_NAME} -lzmq)

add_executable(${PROJECT_NAME}_broker src/brokerZMQ.cpp)
add_executable(${PROJECT_NAME}_client src/clientZMQ.cpp src/MironDDSListener.cpp)

target_link_libraries(${PROJECT_NAME}_broker -lzmq)
target_link_libraries(${PROJECT_NAME}_client -lzmq -pthread -ldcpsgapi -ldcpsgapi -lddsdatabase -lddsos -lddskernel -ldcpsisocpp2 -lroqmedds)

if(GTEST_FOUND)
  enable_testing()
  INCLUDE_DIRECTORIES(tests)
  FILE(GLOB_RECURSE TEST tests/*.cpp)
  ADD_EXECUTABLE(${PROJECT_NAME}Test ${TEST}) 
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}Test GTest::GTest GTest::Main)
endif()
