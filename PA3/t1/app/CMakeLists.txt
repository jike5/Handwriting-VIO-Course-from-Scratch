add_executable(testCurveFitting CurveFitting.cpp)
target_link_libraries(testCurveFitting ${PROJECT_NAME}_backend)

add_executable(myCurveFitting myCurveFitting.cpp)
target_link_libraries(myCurveFitting ${PROJECT_NAME}_backend)