# Path Planner

## Overview 
This project implements a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. The goal for the path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## Build Instructions 
1. Clone the Path Planner git repository
    ```  
    $ git clone https://github.com/jfoshea/Path-Planner.git
    ```
2. This project involves the Term 3 Simulator which can be downloaded here [link](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

3. Build the project using cmake or using the scipts below 
    ```  
    $ ./clean.sh 
    $ ./build.sh 
    ```

4. Run the Path Planner
    1. Launch the simulator and select Path Planner 
    2. Use the run script to connect to the server and run the simulation 
    ```  
    $ ./run.sh
    ```
