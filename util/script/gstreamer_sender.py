#!/usr/bin/env python

# for mac https://sayak.dev/install-opencv-m1/

import cv2
print(cv2.__version__)

# Uncommenting this would allow to check if your opencv build has GSTREAMER support 
#print(cv2.getBuildInformation())


cap = cv2.VideoCapture(0) #, cv2.CAP_GSTREAMER)
#cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! videoconvert ! video/x-raw, format=BGR ! appsink " , cv2.CAP_GSTREAMER)

# For NVIDIA using NVMM memory 
#cap = cv2.VideoCapture("udpsrc port=5000 ! application/x-rtp,media=video,encoding-name=H264 ! queue ! rtpjitterbuffer latency=500 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink drop=1", cv2.CAP_GSTREAMER)

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
#fps = cap.get(cv2.CAP_PROP_FPS) #doesn't work with python in my case so forcing below...you may have to adjust for your case
fps = 30

if not cap.isOpened():
   print('Failed to open camera')
   exit

print('Source opened, framing %dx%d@%d' % (width,height,fps))


writer = cv2.VideoWriter("appsrc ! video/x-raw,format=BGR ! queue ! videoconvert ! x264enc insert-vui=1 ! h264parse ! rtph264pay ! udpsink port=5000", cv2.CAP_GSTREAMER, 0, float(fps), (int(width),int(height))) 
#writer = cv2.VideoWriter("appsrc ! videoconvert ! x264enc ! rtph264pay ! udpsink host=127.0.0.1 port=8553", cv2.CAP_GSTREAMER, 0, float(fps), (int(width),int(height)))
# For NVIDIA using NVMM memory 
#writer = cv2.VideoWriter("appsrc ! video/x-raw,format=BGR ! queue ! videoconvert ! video/x-raw,format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=1 insert-vui=1 ! h264parse ! rtph264pay ! udpsink port=5001", cv2.CAP_GSTREAMER, 0, float(fps), (int(width),int(height))) 

if not writer.isOpened():
   print('Failed to open writer')
   cap.release()
   exit


while True:
    ret_val, img = cap.read();
    if not ret_val:
        break

    writer.write(img);
    cv2.waitKey(1)

writer.release()
cap.release()

"""
cmake \
  -DCMAKE_SYSTEM_PROCESSOR=arm64 \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DWITH_OPENJPEG=OFF \
  -DWITH_IPP=OFF \
  -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D OPENCV_EXTRA_MODULES_PATH=/Users/shinkansan/workspace/opencv_contrib-4.5.0/modules \
  -D PYTHON3_EXECUTABLE=/opt/homebrew/Caskroom/miniforge/base/envs/vdcl_remote/bin/python \
  -D BUILD_opencv_python2=OFF \
  -D BUILD_opencv_python3=ON \
  -D INSTALL_PYTHON_EXAMPLES=ON \
  -D INSTALL_C_EXAMPLES=OFF \
  -D OPENCV_ENABLE_NONFREE=ON \
  -D WITH_GSTREAMER=ON \
  -D WITH_TBB=ON \
  -D WITH_V4L=ON \
  -D WITH_OPENGL=ON \
  -D BUILD_EXAMPLES=ON ..
"""