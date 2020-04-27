PHAS0100Assignment1 Linear Regression
------------------

Purpose
-------

The purpose of this project is to solve a linear regression of the form:

y = theta1*x + theta0

Given a text document which contains a column of x values, and a column of y values.

Credits
-------

This project was developed by Griffin Bassman.

Build Instructions
------------------
If you want to use the VTK visualiser you need the VTK development libraries. On Ubuntu these can be installed with:
``` shell
apt-get install libvtk7-dev

If you would like to build this project from a zip file, please extract PHAS0100Assignment2. From here, open up your terminal and navigate to the same directory that PHAS0100Assignment1 is in and type:

```
mkdir PHAS0100Assignment2-Build
cd PHAS0100Assignment2-Build
cmake ../PHAS0100Assignment2
make
```

At this point, the built project will be in the PHAS0100Assignment1-Build directory.

How to Use
------------------

Once the project has been properly built and you are in the PHAS0100Assignment1-Build directory, type:
```
cd bin
```
In order to get in the same directory as the app. In this directory, you will find two files, TestData1.txt and TestData2.txt which are sample files to perform a linear regression. You can either perform a linear regression using the normal equation or gradient descent. If you would like the use the normal equation, type:
```
./lrgLinReg -n [file_name.txt]
```
If you would like to use the gradient descent method, type:
```
./lrgLinReg -g [file_name.txt] [epochs] [learning_rate] [theta0_guess] [theta1_guess]
```
Where [epochs] specifies the number of iterations, [learning_rate] specifies the learning rate of the algorithm, and [theta0_guess] and [theta1_guess] specify initial values for these parameter. If you would like more details please type:
```
./lrgLinReg [-h/--help]
```
Additionally, this directory contains a executable to run all of my tests. To run this type:
```
./lrgLeastSquaresSolverTests
```
Please check out the file Screenshot_of_test_results.pdf on the cover to see some example commands and expected output. 
Enjoy!
