# Kidnapped Vehicle Project

This project consists of a c++ implementation of a [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter) to estimate the location (GPS position) of a moving car. This project improves the estimations by sensing landmarks and associating it with their actual positions obtained from a known map. 

The main goals of this project is to develop a c++ Particle filter that successfully estimates the position of the car from the Udacity Simulator. Figure 1 depicts an example of the filter estimating (blue circle) the object position. The RMSE (Root Mean Square Error) values estimates the accuracy of the Particle Filter Estimation.

![alt text][image4]

[//]: # (Image References)

[image1]: images/success.png "Particle Filter Correctly Estimates Car Location" 
[image2]: images/10_particles.png "Particle Filter with 10 particles." 
[image3]: images/50_particles.png "Particle Filter with 50 particles." 
[image4]: images/animation.gif "Particle Filter estimating car locations." 



## 1.Access 

The source code for this project is available at [project code](https://github.com/otomata/CarND-Kidnapped-Vehicle-Project).

## 2.Files

The following files are part of this project: 
* particle_filter.cpp:   Particle Filter class definition;
* map.cpp: Map class definition;
* main.cpp: Main file; 
* images: 
** success.png:  Location Estimation using the Udacity Simulator; 

### Dependency

This project requires the following packages to work:
* Udacity Simulator [https://github.com/udacity/self-driving-car-sim/releases/](https://github.com/udacity/self-driving-car-sim/releases/);
* cmake 3.5 or above;
* make 4.1 or above;
* gcc/g++: 5.4 or above;
* uWebSocketIO;

### WebSocketIO

This project uses the open source package called WebScokectIO to facilitate the communication between the 
Particle Filter and the Udacity Simulator. To install all the websocketio libs, execute the script ``install-ubuntu.sh`` from the project repository directory.

## 3.How to use this project

To run this project, you first need to compile the code. After the code is compiled, please, run the Udacity simulator and the ``particle_filter`` binary created at the build folder.

### Compiling and Running

The main program can be built and run by doing the following from the project top directory.

1. ./build.sh
2. cd build
3. ./particle_filter
4. Run the Udacity Simulator

## 4.Results

Figure 2 depicts the Particle Filter estimations using 10 particles. The RMSE values of 0.154, 0.122 and 0.005 show the Particle Filter accuracy to estimate the car's location (x and y) and yaw angle (bearing). 

![alt text][image2]

Figure 3 shows the Particle Filter estimations using 50 particles. Increasing the number of particles improves the filter accuracy.

![alt text][image3]





