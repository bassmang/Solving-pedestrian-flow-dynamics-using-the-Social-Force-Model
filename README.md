PHAS0100Assignment2 Solving Pedestrian Flow Dynamics Using the Social Force Model
------------------

Purpose
-------

The purpose of this project is to model pedestrian movement based on Social Force Model (SFM) for pedestrian dynamics from [Helbing & Molnar Phys. Rev. E 51, 4282 (1995)]. This project provides a high degree of flexibility in producing accurate models of pedestrian movement and provides visualisations.

Credits
-------

This project was developed by Griffin Bassman.

Build Instructions
------------------
If you want to use the VTK visualiser you need the VTK development libraries. On Ubuntu these can be installed with:
``` shell
apt-get install libvtk7-dev
```
If you would like to build this project from a zip file, please extract PHAS0100Assignment2. From here, open up your terminal and navigate to the same directory that PHAS0100Assignment2 is in and type:

```
mkdir PHAS0100Assignment2-Build
cd PHAS0100Assignment2-Build
cmake ../PHAS0100Assignment2
make
```

At this point, the built project will be in the PHAS0100Assignment2-Build directory.

How to Use
------------------

Once the project has been properly built and you are in the PHAS0100Assignment2-Build directory, type:
```
cd bin
```
In order to get in the same directory as the app. In this directory there are a number of applications which you can use to visualise pedestrian movement. Here is an explanation of all the commands I implemented:
```
./sfmPedestrianTest
```
This will run a suite of unit tests to ensure the inner functionality of my program is working correctly. All 39 assertions should pass
```
./sfmPedestrianFlowModel
```
This is a simple model with 3 pedestrians that tests the funcionality of Part 4. Initially I printed out relevant values for this model, however I updated it with the visualisation tool to show the functionality more clearly.
```
./sfmTwoSidesModel
```
This is the model described in Part 5. Here I initialized 5 pedestrians on left side, and 5 on the right, each with their destination on the other side. I have included screenshots from this program at t=0 and t=10 named as follows:
<br />
"TowSided Model, t=0.png" <br />
"TowSided Model, t=10.png
```
./sfmBoxToTargetModel
```
This is the model described in Part 6(c). I initialised 20 targeted pedestrians in the top right corner with all of their destinations in the bottom left corner. I have included screenshots from this program at t=0 and t=10 named as follows:
<br />
"BoxToTargetModel, t=0.png" <br />
"BoxToTargetModel, t=10.png"
```
./sfmBoxToTargetWithDirectionalModel
```
This is the model described in Part 6(d). I initialised 10 targeted pedestrians in the top right corner with all of their destinations in the bottom left corner. I also initialised 10 directed pedestrians on the left side, all with a direction to move to the right. I have included screenshots from this program at t=0 and t=10 named as follows:
<br />
"BoxToTargetWithDirectionalModel, t=0.png" <br />
"BoxToTargetWithDirectionalModel, t=0.png"
```
./sfmBenchmarkStandard
```
This program is used for benchmarking purposes as described in Part 7. It runs a similar model to that in Part 6(d), but uses 200 pedestrians instead of 20, and does not include visualisations. This runs without parallelisation and prints out the total CPU time as well as Wall time to run.
```
./sfmBenchmarkParallel [num_threads]
```
This runs a similar program as above, but uses parallelisation. This program uses two separate parallel loops, one to calculate the force on each pedestrian, and another to update the velocity/position of each pedestrian. It also allows the user to input the number of threads to use. Finally, it will print out the total CPU, total Wall time, and number of threads used.

Benchmarking
------------------

Using the two programs above, I was able to benchmark the performance of my program based on whether parallelisation was used, and if so how many threads were used. I have included a graphic showing how the number of threads affected runtime named:
<br />
"BenchmarkPerformance.png" <br />
