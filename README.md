# Setting up cppVrepLKAS in Linux
A software-in-the-loop simulator for lane keeping assist system (LKAS) using Vrep and C++. 
If you are using this framework please refer to the following paper:

`[1] S. Mohamed, S. De, K. Bimpisidis, V, Nathan, D. Goswami, H. Corporaal, and T. Basten, “IMACS: A Framework for Performance Evaluation of Image Approximation in a Closed-loop System,” in: MECO, 2019.`

The controller is designed in Matlab and the control matrices `\Phi`, `\Gamma`, and control gains `K` (and `F`) are copied to the `src/LaneDetection_and_Control/lateralcontrol_multiple.cpp` file. The LKAS controller is designed as explained in the following papers:
```
[2] S. Mohamed, A. U. Awan, D. Goswami, T. Basten, "Designing image-based control systems considering workload variations," in: CDC, 2019.
[3] S. Mohamed, D. Zhu, D. Goswami, T. Basten, "Optimising quality-of-control for data-intensive multiprocessor image-based control systems considering workload variations," in: DSD, 2018.
```

An initial framework using Vrep and Matlab has been presented in:
```
S. Mohamed, D. Zhu, D. Goswami, T. Basten, "DASA: an open-source design, analysis and simulation framework for automotive image-based control systems," in: MCAA Annual Conference, 2019.
```

Tested using the following versions. If you would like to use latest versions, you might have to edit the syntax of some functions.
Tested with the following versions:
* OS: Ubuntu 18.04
* g++ (Ubuntu 7.4.0-1ubuntu1~18.04.1) 7.4.0

1. Install dependent libraries
2. Install OpenCV
3. Install Eigen
4. Install VREP
5. Run IMACS_LKAS

It is suggested that you Fork [this](https://github.com/sajid-mohamed/cppVrepLKAS) repository by clicking the "fork" button in the top right. This lets you have your own working copy.

Clone the forked (or directly [this](https://github.com/sajid-mohamed/cppVrepLKAS)) repository:
```
git clone https://github.com/sajid-mohamed/cppVrepLKAS.git
cd cppVrepLKAS
pwd
```
For brevity, `$(root)=pwd`, i.e. the path to `cppVrepLKAS` is called as `$(root)`. 

# 1.-4. Dependencies

## 1. Dependent libraries
The following libraries might be needed for successful execution.
```
sudo apt-get install libtinfo-dev
sudo apt-get install libjpeg-dev
sudo apt-get install libtiff5-dev
sudo apt-get install libpng-dev
sudo apt-get install jasper
sudo apt-get install libgtk-3-dev
sudo apt-get install libopenblas-dev
sudo apt-get install -y libavcodec-dev
sudo apt-get install -y libavformat-dev
sudo apt-get install libavutil-dev
sudo apt-get install libswscale-dev
sudo apt-get install valgrind
# For openCV installation
sudo apt-get install cmake
## qt4
sudo apt-get install qt4-qmake
sudo apt-get install libqt4-dev
```
## 2. Installing OpenCV

If opencv is already installed, go to step `2.5`.

If not installed, follow the OpenCV installation steps from https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html.

In case you have errors with `ccache` do `2.1`.

If you have issues with `java` (e.g. if there is an error: unable to locate `*.jar`) do `2.2`.

After `2.1` or `2.2` you can continue from `2.3`.

### 2.1 Install ccache
```
sudo apt install -y ccache
sudo /usr/sbin/update-ccache-symlinks
echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc
source ~/.bashrc && echo $PATH
```
### 2.2 Install java
You can follow the tutorial here: https://www.digitalocean.com/community/tutorials/how-to-install-java-with-apt-on-ubuntu-18-04.

In case there is an error: unable to locate `*.jar`:
```
sudo apt install default-jre
sudo apt install default-jdk
sudo update-alternatives --config javac
sudo update-alternatives --config java
```
Choose the correct `jdk` version. `jre` does not have `tools.jar`
Update the JAVA_HOME path
```
sudo gedit /etc/environment
```
At the end of this file, add the following line, making sure to replace the path with your own copied path:
```
JAVA_HOME="/usr/lib/jvm/java-11-openjdk-amd64/bin/"
```
Modifying this file will set the JAVA_HOME path for all users on your system.
Save the file and exit the editor.

Now reload this file to apply the changes to your current session:
```
source /etc/environment
```
If you do not have `sudo` access, follow the steps in `FAQ 4`.

### 2.3 Getting OpenCV Source Code
```
cd $(root)/externalApps
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```
`externalApps` is a folder already available when you `git clone cppVrepLKAS`. It might be better to include all the external apps in this folder.

### 2.4 Building OpenCV from Source Using CMake
```
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. -DOPENCV_GENERATE_PKGCONFIG=ON
make
sudo make install
```
Note the path of `opencv*.pc` file. This is the `PKG_CONFIG_PATH`. 
Typically this should be `/usr/lib/pkgconfig`. If you are using virtual machine this might be `/usr/local/lib/pkgconfig`.

### 2.5 `opencv.pc` should be in the `PKG_CONFIG_PATH`
Make sure `opencv.pc` file is in the `/usr/lib/pkgconfig` or change the `PKG_CONFIG_PATH` to point to this file. To change path follow the steps in `FAQ 3` or `FAQ 4` depending on whether you have `sudo` rights.
Remember the path to your `opencv` installation as `OPENCV_PATH`.

To check if `opencv.pc` is in the `/usr/lib/pkgconfig`:
```
cd /usr/lib/pkgconfig
[ -f /usr/lib/pkgconfig/opencv.pc ] && echo "File exist"
```
You could also use `dlocate` to find the file.

If you cannot find it, try:
```
sudo apt-get install libopencv-dev
```

## 3. Install eigen
```
cd $(root)/externalApps
git clone https://gitlab.com/libeigen/eigen.git
```
The path to eigen is needed later for `EIGEN_PATH`.
## 4. Install Vrep (CoppeliaSim)
Download the linux version from https://coppeliarobotics.com/downloads. 

The license to CoppeliaSim should be obtained from their website. 

If you are in an academic institution, you could simply download the `Edu` version.

Make sure you copy the downloaded file (e.g. `CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz`) to the `$(root)/externalApps` folder.
```
cp CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz $(root)/externalApps
cd $(root)/externalApps
mkdir vrep
tar -xvf CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz -C vrep --strip-components=1
```

Generally for local tar file:
```
tar -xvf FILENAME -C FOLDER --strip-components=1
```
The `--strip-components` flag is used when a tar file would naturally expand itself into a folder, let say, like github where it examples to `repo-name-master` folder. Of course you wouldn’t need the first level folder generated here so `--strip-components` set to 1 would automatically remove that first folder for you. The larger the number is set the deeper nested folders are removed.

The `VREP_PATH` is then `$(root)/externalApps/vrep`.

# 5. Running cppVrepLKAS

## 5.1 Change hardcoded paths
Please change the paths in the following files to the corresponding paths in your own system.
```
$(IMACSROOT)/cppVrepLKAS.pro
$(IMACSROOT)/src/LaneDetection_and_Control/lane_detection.cpp
```
1. If the external applications vrep, eigen or opencv are installed to another directory than the `$(ROOT)/externalApps`, change the corresponding `OPENCV_PATH`, `PKG_CONFIG_PATH`, `EIGEN_PATH`, and `VREP_PATH` in `cppVrepLKAS.pro` to the corresponding PATH in your system. 

To obtain the path, you could run `pwd` in a terminal opened from the corresponding folder.

2. Change line 89 of `lane_detection.cpp` to `string out_string = "$(ROOT)/out_imgs";` where $(IMACSROOT) is your actual path.

## 5.2 Run bash script and follow the instructions to check if the framework is working
```
cd $(ROOT)
bash run.sh
```
You can change the commands in this file to run automatically.

In case the `run.sh` does not work, do steps `5.3 - 5.5`.
The steps included in the bash script are the following:

## 5.3 Make imacsLKAS
Generate the `Makefile` using `qmake`.
```
cd $(ROOT)
qmake cppVrepLKAS.pro
make
```

When you want to run `make` in `$(ROOT)` always run `qmake cppVrepLKAS.pro` first. 

`make clean` cleans the files generated by this `make`.

## 5.4 Start a vrep scene
Open a vrep-scene. Vrep needs to be started before running IMACS framework.
```
cd $VREP_PATH
./coppeliaSim.sh ../../vrep-scenes/SmallBias_report.ttt
```
If you are using an older version of vrep (`coppeliaSim.sh is not found`), please use the following command:
```
./vrep.sh ../../vrep-scenes/EnterCurveTest.ttt
```
## 5.5 Run imacsLKAS (in a new terminal)
```
cd $(ROOT)
# Usage: ./imacsLKAS {version} {simstep}
# Minimum one argument needed. (default values) ./imacs 1 0.005
./imacsLKAS 1
```
There might be some dialog windows opening in `VREP` if you are running it for the first time. This is normal.

You do not have to always close vrep to run imacs again. Just `stop simulation` in vrep and wait for sometime before executing `./imacsLKAS 1 0.005`.

In case you encounter some other errors, you can debug using the following command:
```
valgrind ./imacsLKAS 1
```

# 6. How to use imacsLKAS?

The main file: `src/cpp_vrep_api/cpp_vrep_framework.cpp`
* The simulation step time, the total simulation time, the initial wait time, the sampling period(s) and delay(s) are hardcoded in this file.
* If you want to add additional input arguments for `imacsLKAS`, you should do this here.

Controller Design: `src/LaneDetection_and_Control/lateralcontrol_multiple.cpp`
* The controller matrices are stored in this file. `\Phi`, `\Gamma` and `K` for multiple controllers are stored in the variables `m_phi_aug`, `m_Gamma_aug` and `m_K2c`. 
* The controller currently used is an optimal LQR control for a regulation problem where the objective is for the output to reach zero. If you want to change the type of controller you can edit the function `lateralController::compute_steering_angles()`.

Image Processing: `src/LaneDetection_and_Control/lane_detection.cpp`
* This file contains the image processing steps. Currently, we do the steps mentioned in [1]. You should be changing the functions in this file to include your own image processing algorithm.

# FAQs
## 1. When running `make` in `$(IMACSROOT)`: the following warnings (many) are also observed by us and is normal. We are working on improving this in a later version.
```
matrix.hpp [-Wreorder]
GivensQR.hpp [-Wreorder]
extApiPlatform.c [-Wunused-result]
```
## 2. If you receive the following errors, wait for some time before running the command `./imacsLKAS 1` If that does not work, close and reopen vrep.
```
$(ROOT)/src/cpp_vrep_api/my_vrep_api.cpp:59: vrepAPI::vrepAPI(): Assertion `m_clientID != -1 && "V-REP must be started: ./vrep.sh ../../vrep-scenes/EnterCurveTest.ttt"' failed.
```
```
OpenCV Error: Assertion failed (mv && n > 0) in merge, file /home/om/hd/infra/build_opencv/opencv/modules/core/src/convert.cpp, line 327
terminate called after throwing an instance of 'cv::Exception'
  what():  /home/om/hd/infra/build_opencv/opencv/modules/core/src/convert.cpp:327: error: (-215) mv && n > 0 in function merge
```
## 3. Exporting paths permanently with sudo rights
To set paths globally if you have sudo rights:
```
sudo gedit /etc/environment
```
At the end of this file, add the following line, making sure to replace the path with your own copied path:
```
PKG_CONFIG_PATH=/usr/lib/pkgconfig/
```
If there were already variables present with the same name, append the above variables with a `:`.
Modifying this file will set the variables for all users on your system.
Save the file and exit the editor.
Now reload this file to apply the changes to your current session:
```
source /etc/environment
```
## 4. Exporting paths without sudo rights
If you want to export paths permanently,
Open a terminal window using `Ctrl+Alt+T`
Run the command 
```
gedit ~/.profile
```
Add the line(s) to the bottom of this file and save.
```
export PKG_CONFIG_PATH=/usr/lib/pkgconfig/
```
Log out and log in again.
A safer way is to use quotes. Doing so is necessary if one or more directories in the original PATH contain spaces. E.g.:
```
export CLANG="$(root)/clang+llvm-9.0.0-x86_64-linux-gnu-ubuntu-18.04/bin/clang"
```
Alternatively, you can set the paths in `paths.sh` and then source paths
```
cd $(IMACSROOT)
source paths.sh
```
## 5. The simulation time is hardcoded to 15 in src/cpp_vrep_api/cpp_vrep_framework.cpp. This may result in erroneous behaviour after the simulation exceeds this time.
Change line 11 in `src/cpp_vrep_api/cpp_vrep_framework.cpp` if you want to simulate for a longer time, i.e. `int simulation_time = 100;` if you want to logically simulate for `100 sec`.

## 6. How to start a straight road scene?
To start a straight road scene (see `vrep-scenes/SmallBias_report.ttt`), replace line 7 in `run.sh` with the following
```
gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep-scenes/SmallBias_report.ttt; exec bash'
```

## 7. Points to remember if running the framework using `bash run.sh`
If you are making changes to the `cppVrepLKAS.pro` file, you would have to run the command `make clean` first in the terminal before running `bash run.sh`.
```
make clean
bash run.sh
```
## 8. Control design: relation of matlab control codes in [1],[2] and [3] to the framework
The `lateralcontrol_multiple.cpp` has a sample controller implemented. 

You are required to adapt the codes according to the control design you choose (especially the `lateralController::compute_steering_angles()` function). Ideally, you should rewrite the `lateralController::compute_steering_angles()` function.

| Matlab variable | Framework equivalent |                                   Files to change                                   |
|:---------------:|:--------------------:|:-----------------------------------------------------------------------------------:|
|       `phi`       |       `m_phi_aug`      | `src/laneDetection_and_Control/lateralcontrol_multiple.cpp`, `include/config.hpp:76-83` |
|     `Gamma`       |     `m_Gamma_aug`      | `src/laneDetection_and_Control/lateralcontrol_multiple.cpp`, `include/config.hpp:76-83` |
|        `K`        |         `m_K2c`        | `src/laneDetection_and_Control/lateralcontrol_multiple.cpp`, `include/config.hpp:76-83` |
|       `T`         |        `m_T`           | `src/laneDetection_and_Control/lateralcontrol_multiple.cpp`, `include/config.hpp:76-83` |
|        `h`        |       `period_s`       |                      `src/cpp_vrep_api/cpp_vrep_framework.cpp:40`                     |
|       `tau`       |        `delay_s`       |                      `src/cpp_vrep_api/cpp_vrep_framework.cpp:41`                     |

## 9. Some variable descriptions

| Variable |Description |                                   Files to change                                   |
|:---------------:|:--------------------:|:-----------------------------------------------------------------------------------:|
|       `simstep`       |      Simulation step time: calculated as a greatest common factor between h and tau. A really low value would slow down the simulation.      | `src/cpp_vrep_api/cpp_vrep_framework.cpp:10` |
|       `simulation_time`       |      Total simulation time: total logical time for which you want to run the simulation.     | `src/cpp_vrep_api/cpp_vrep_framework.cpp:11` |
|       `wait_time`       |      Initialisation time for which the controller is inactive. May be needed to achieve required vehicle speed or for getting the initial frame. (set this to 2.5 seconds, if getting errors).    | `src/cpp_vrep_api/cpp_vrep_framework.cpp:12` |
