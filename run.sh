#!/bin/sh
read -p "Please set correctly the VREP_PATH, OPENCV_PATH and EIGEN_PATH in cppVrepLKAS.pro first. Press enter after setting the paths."
source /etc/environment
source paths.sh
qmake cppVrepLKAS.pro
make
gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep-scenes/EnterCurveTest.ttt; exec bash'
read -p "Press enter after the vrep scene has been opened"
./imacsLKAS 1 0.001

