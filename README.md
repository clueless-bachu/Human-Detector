# Human Detection and Tracking
[![Build Status](https://travis-ci.org/clueless-bachu/Human-Detector.svg?branch=master)](https://travis-ci.org/clueless-bachu/Human-Detector)
[![Coverage Status](https://coveralls.io/repos/github/clueless-bachu/Human-Detector/badge.svg?branch=master)](https://coveralls.io/github/clueless-bachu/Human-Detector?branch=master)
---

## Overview & Description

The  ability  of  a  robot  to  understand  the  environment  is  vital  for  autonomy.   Our  product  is  aHuman Detection and Tracking software.  The ability to detect and track humans opens up multipleopportunities for robots to imbibe reactive behaviours that can potentially lead to better Human-Robot Interaction, safety protocols, efficient path planning etc.

Main Features:
* Detection and Tracking Module
* Depth Estimation Module
* 3D Transfomation Module
* Image Preprocessing Module
* IOHanlder Module
* Robot Module that shows demo of Perception Module Integration with Robot.

3 Minute Video for Phase 1
[Video Link](https://www.youtube.com/watch?v=N3DkdJLmnMI&feature=youtu.be)

## Personnel

### Phase 2
* Vasista Ayyagiri (clueless-bachu) - Navigator
* Vishnuu AD (vishnuu95) - Driver

### Phase 1
* Vasista Ayyagiri (clueless-bachu) - Driver
* Vishnuu AD (vishnuu95) - Navigator

## Dependencies

* OpenCV
* Google Test
* Google Mock
* Eigen3

## Standard install via command-line
```
git clone --recursive https://github.com/clueless-bachu/Human-Detector.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

## Product Backlog File

Google Doc Planning Sheet - https://docs.google.com/document/d/1vK4eTWuJDBQswQ49AHXX5LzxAnbU5EcfVC7Dlu_3VPk/edit?usp=sharing

AIP Sheet - https://docs.google.com/spreadsheets/d/1YPsMStteSjXmPFDnUeONrdbhdRGQZ498H-M4ctVbVvw/edit?usp=sharing

## TODO

* ~~Design UML diagrams and Activity Flow~~
* ~~Download and install required technologies~~
* ~~Declare all classes~~
* ~~Add stub Functions for each class~~
* ~~Create pipeline for detection and tracking~~
* ~~Commenting and Doxygen Documenting~~
* ~~Add unit test cases~~
* Pass all Static testing cppcheck, cpplint, travis and coveralls
* ~~Identify DL Model Source Repositories~~
* PHASE 2 - Implementation of Classes
* ~~Change Quadchart from PNG to PDF~~
* Update travis yml for successfull build.

## Iteration 2 requests
~~* Class related changes:~~
~~  * 1) IO handler:~~
~~    * Add parsing for model config path, model weights path, human height, image height, image width, camera intrinsics, camera to robot transformation.~~
~~    * Remove all video related attributes and functions as it might cause error.~~
~~    * Give access to all private variables via public functions.~~
~~  * 2) Depth Estimation:~~
~~    * Remove all attributes and add them as arguments to function.~~
~~  * 3) DetTrack:~~
~~    * Add attributes to store thresholds, image params and detected objects.~~
~~    * Modify add trackers to take in input image.~~
~~* Unit Test related changes.~~
~~  * 1) Add test cases for new ioh functions.~~
  ~~* 2) Init test case for Robot class is reduntant - could remove it.~~
  ~~* 3) Change arguments input to IOU from x1,y1,w,h to x1,y1,x2,y2 in det and tracking test case.~~
* ~~UML Diagrams: Update UML diagrams according to above changes.~~
* Fix travis build errors.
~~* It seems like eigen is causing build issues in travis. Lets substitute eigen. Update header files and implementation.~~

## Known Issues

~~* Need to fix Complier Warnings.~~
