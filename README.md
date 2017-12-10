# Kidnapped Vehicle Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This repository contains my submission for the Kidnapped Vehicle Project, which is part of the curriculum for term 2 of the Udacity Self-Driving Car Engineer Nanodegree Program. The goal of this project is to build a particle filter in C++ and combine it with map data to localize a vehicle. 

[//]: # (Image References)

[image1]: ResultScreenShot.png "Result Screenshot"
[image2]: ErrorvsNumParticles.png "Error vs # of Particles"

![alt text][image1]

### Files Submitted
The source code for my project can be found [here](./src/). All of my changes can be found in the implementation of the [ParticleFilter class](./src/particle_filter.cpp). 

### Results
Looking at the screenshot of the Udacity simulator above you can see that my Particle filter satisfied the requirements of the assignment. The results shown in the screenshot were generated using 100 particles. I was curious to see how the results and performance would vary as the number of particles were reduced. My results for these experiments can be seen in the table below.  

| Num Particles  | Error X  |  Error Y | System Time | 
|:-----:|:-------------:|:-------:|:---------:|
| 5  | 136.194  | 147.668 |  55.82 |
| 6  | 136.711  | 145.494 |  56.96 |
| 7  | 0.161  | 0.170 |  55.74 |
| 8  | 0.159  | 0.170 |  54.80 |
| 10  | 0.159  | 0.136 |  55.62 |
| 20  | 0.134  | 0.131 |  56.16 |
| 40  | 0.126  | 0.113 |  49.02 |
| 50  | 0.123  | 0.113 |  55.68 |
| 100  | 0.109  | 0.102 |  56.70 |
| 200  | 0.113  | 0.102 |  53.92 |
| 500  | 0.108  | 0.100 |  58.14 |

I determined that the system time data, as it is reported by the test application, does not directly correlate to computational time. An examination of the algorithm shows that all of the methods in the ParticleFilter class with the exception of the dataAssociation method (which is not computationally intensive) iterate over all of the particles in their outermost loop. Hence the computational time should vary linearly with the number of particles. The fact that it always takes close to one minute leads me to believe that the  system time is not being reported correctly. 

The error varies exponentially with the number of particles. Visualizing the data in the shown below, can see that the error increases rapidly as a number of particles less than 20 is used. But levels out rapidly and shows diminishing returns after 100. Hence, I chose 100 as the final number of particles for my project.

![alt text][image2]
