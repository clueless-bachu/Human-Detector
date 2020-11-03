set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp  
    detectAndTracking.cpp
    estimatingDepth.cpp  
    ioHandling.cpp 
    preprocessing.cpp  
    robot.cpp 
    transforming.cpp
    ../app/depthEstimator.cpp  ../app/detTrack.cpp  ../app/ioHandler.cpp  ../app/preprocess.cpp  ../app/robot.cpp  ../app/transformation.cpp
)

target_include_directories(cpp-test PUBLIC 
../vendor/googletest/googletest/include ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest ${OpenCV_LIBS})