project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")


set(PREDICTOR_SRCS prediction/include/pred_helpers.h prediction/include/classifier.h prediction/classifier.cpp)
add_library(prediction ${PREDICTOR_SRCS})
target_include_directories(prediction PUBLIC prediction/include)


include_directories(include)
include_directories(vehicle/include)
include_directories(object_tracker/include)
set(sources src/main.cpp include/helpers.h include/json.hpp vehicle/vehicle.cpp object_tracker/tracker.cpp)
# set(sources src/main.cpp include/helpers.h include/json.hpp prediction/include/pred_helpers.h prediction/include/classifier.h prediction/classifier.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS prediction)
