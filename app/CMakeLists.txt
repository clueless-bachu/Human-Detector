add_executable(shell-app main.cpp depthEstimator.cpp  detTrack.cpp  ioHandler.cpp preprocess.cpp  robot.cpp  transformation.cpp )
target_link_libraries( shell-app ${OpenCV_LIBS} Eigen3::Eigen)

#add_executable(tester tester.cpp ioHandler.cpp depthEstimator.cpp)
#target_link_libraries( tester ${OpenCV_LIBS} Eigen3::Eigen)

#add_executable(detector detTrack.cpp)
#target_link_libraries(detector ${OpenCV_LIBS} Eigen3::Eigen)


include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
