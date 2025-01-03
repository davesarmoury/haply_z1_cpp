cmake_minimum_required(VERSION 3.26)
project(haply_z1_cpp LANGUAGES CXX)

include_directories(include)

link_directories(${PROJECT_SOURCE_DIR}/lib)
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} Haply.HardwareAPI)

install(TARGETS ${PROJECT_NAME})
