CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( USEOMEGA)

include_directories("/usr/include/eigen3")
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
add_executable(useOmega useOmega.cpp)
target_link_libraries(useOmega ${Sophus_LIBRARIES})