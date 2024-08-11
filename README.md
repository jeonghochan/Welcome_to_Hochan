## fmtc_remote

## Haptic
### Structure
1. `g29_force_feedback.cpp` -> main file for calc torque and control
2. make shared object (*.so) file by 
``` bash
g++ -c -fPIC g29_force_feedback.cpp -o g29_lib_shinkansan.o
g++ -shared  g29_lib_shinkansan.o -o g29_lib_shinkansan.so
```
3. `g29_adapter.py` for load so file and make python-class(ify) 
4. Enjoy (you can get a hint from adapter.py for usage)

### Self Alignment Torque (Default On)
# fmtc_remote


### Requirements
1. `requirements.txt` contains required python packages
2. it needs gstreamer compliled opencv
3. python3 rospkg, catkin_pkg are needed for carla supports


### Compile OpenCV with Gstreamer

```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D PYTHON_EXECUTABLE=$(which python3) \
-D BUILD_opencv_python2=OFF \
-D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") \
-D PYTHON3_EXECUTABLE=$(which python3) \
-D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
-D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
-D WITH_GSTREAMER=ON \
-D BUILD_EXAMPLES=ON ..

```
