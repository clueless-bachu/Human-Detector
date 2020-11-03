/**
 *  @file    main.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief Main function to demo the functionalities of the Perception Module- Human Object detector. 
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include "robot.hpp"

using std::cout;
using std::endl;

using robot::Robot;

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Provide a path to the cfg file" << endl;
        return -1;
    }

    Robot robot(argv[1]);
    robot.processData();

    return 0;
}
