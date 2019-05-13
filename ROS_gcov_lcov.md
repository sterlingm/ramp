ROS Gcov and Lcov Implimentation
================================
---

## pre-requisites
This section will go over how to get ROS and gtest if the user has not already.
This guide also assumes the user is using a Unix system.(Unbuntu 16.04 + ..)

## ROS

1. ROS install [ROS]: http://wiki.ros.org/Installation/Ubuntu
2. Follow ROS tutorials if you need to familiarize yourself.
3. Set up a catkin_ws and a ROS project (example can be found in ROS wiki)

## Gtest

1. Gtest install [Gtest]: https://gist.github.com/Cartexius/4c437c084d6e388288201aadf9c8cdd5
2. Follow the steps in the link provided 
3. Go to ROS catkin_ws use catkin_make
---

## Gcov and Lcov
* Gcov is a googletest coverage tool that loops through test code and calculates % 
of lines used.
* Lcov is a front end to Gcov that converts gcov reports to html and provides a user interface.

## Assumptions
* A ROS workspace and project is already created.
* The user is using c++ and googletest.
* The user has code and test code created.

* Code compiles and builds with
```sh
$ catkin_make
```
* Test build with
```sh
$ catkin_make run_tests
``` 
---
## Implementation

1. Install lcov this will also install gcov
```sh
$ sudo apt-get install lcov
$ lcov --directory ./build/ --capture --output-file ./code_coverage.info -rc lcov_branch_coverage=1
```
This will dowload gcov and set up build space. 

2. update, upgrade, install
```sh
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install upgrade
```
This will update your bash with the new gcov function.

3. Turn on ROS in a seperate terminal
```sh
$ roscore
```

4. run test(see Assumptions)
5. Create gcov executable
```sh
$ g++ -o code  -fprofile-arcs -ftest-coverage code_test.cpp -L /usr/lib -I/usr/include
```
This will generate an executable with coverage flag enabled so next you need to run the executable to generate files required for coverage report generation

6. Run executable
```sh
$ ./code
```
This will generate 2 files that are needed to then generate a coverage report.

7. To generate report you need to pass your test file to gcov.
```sh
$ gcov code_test.cpp
```
This now generates many files and will perform the coverage report that is readable in terminal. lcov will make use of these files to generate the GUI in html format and make the data more human readable.

8. pass data into lcov
```sh
$ lcov --directory ./build/ --capture --output-file ./code_coverage.info -rc lcov_branch_coverage=1
```

9. Now we have coverage info lets display it in html file using the command below.
```sh
$ genhtml code_coverage.info --branch-coverage --output-directory ./code_coverage_report/
```
the above command will generate html reports in output directory

Once it is done, you can go through the out folder and open index.html to check coverage percentile. It will display both Line coverage and Function coverage as below.
---
## Usefull site with example and further step by step
Site: https://medium.com/@naveen.maltesh/generating-code-coverage-report-using-gnu-gcov-lcov-ee54a4de3f11











