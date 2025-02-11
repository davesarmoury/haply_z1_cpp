cmake_minimum_required(VERSION 3.16)
project(haply_z1_cpp LANGUAGES CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14 -pthread")

include_directories(include)
link_directories(lib)

add_executable(
    ${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp 
)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} Haply.HardwareAPI Z1_SDK_${CMAKE_SYSTEM_PROCESSOR})

install(TARGETS ${PROJECT_NAME})
