# nerf2face
Software for a Nerf gun robot that detects and shoots at the users face.
Safety goggles are recommended.

The software runs on Raspberry 4 and is based on ROS 1 Melodic.

# Requirements
HW Requirements:
* Raspberry Pi 4
* Raspberry Pi Camera V2.1
* Google Coral TPU: For running fac
* Servo / PWM Pi Hat from Adafruit: For controlling the servos
* 16 bit 12C ADC: For servo voltage feedback
* 2 HITec D485 HW Digital Servos: For aiming, hacked to read voltage
* 1 MG90S Micro Servo: For pulling trigger

SW Requirements:
* Raspbian 10 (buster)
* OpenCV 3.4.7 (guide below)
* OpenCV contrib 3.4.7
* ROS Melodic (guide below)
* Python 2.7.16
* Python deps (guide below)
* Boost

## Install OpenCV with Contrib
Clone `opencv` and `opencv_contrib` alongside each other in a directory.
Untested:
```
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE     -D CMAKE_INSTALL_PREFIX=/usr/local     -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules  -DCPU_BASELINE=NEON,VFPV3 -D ENABLE_NEON=ON -D ENABLE_VFPV3=ON    -D BUILD_TESTS=OFF     -D INSTALL_PYTHON_EXAMPLES=OFF     -D BUILD_EXAMPLES=OFF -DPYTHON_DEFAULT_EXECUTABLE=`which python3` ..
sudo make install
```

## Install ROS Melodic
Untested:
```
mkdir ros_ws
cd ros_ws
../install_ros_melodic.sh
```

## Python deps
```
pip install -r requirements.txt
```

# Build
```
catkin_make
source devel/setup.bash
```

# Run
```
roslaunch nerf2face nerf2face.launch
```
