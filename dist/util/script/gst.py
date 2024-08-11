import cv2
import numpy as np
import time

width = 640
height = 480
fps = 24
# # ! video/x-gdp, format=BGR, width=640, height=480,framerate=24/1 \
# gst_sender_writer = cv2.VideoWriter("appsrc \
# ! videoconvert \
# ! x264enc insert-vui=true bitrate=1000 tune=zerolatency  \
# ! h264parse \
# ! gdppay \
# ! tcpclientsink host=1.249.212.151 port=6000 sync=false", cv2.CAP_GSTREAMER, 0, float(fps), (int(width),int(height)), True)

gst_sender_writer = cv2.VideoWriter("appsrc \
! videoconvert \
! x264enc tune=zerolatency byte-stream=true bitrate=10000000 \
! h264parse config-interval=1 \
! rtph264pay \
! udpsink host=1.249.212.151 port=5118", cv2.CAP_GSTREAMER, 0, float(fps), (int(width),int(height)), True)


gst_str_rtp = "appsrc ! videoconvert ! x264enc noise-reduction=10000 tune=zerolatency byte-stream=true threads=4 " \
              " ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000"
frame_width = 640
frame_height = 480

# out = cv2.VideoWriter(gst_str_rtp, 0, fps, (frame_width, frame_height), True)


def test_image():
    img = np.zeros((480,640,3), np.uint8)
    cv2.putText(img, str(time.time()), (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
    return img

while 1:
    img = test_image()
    cv2.imshow("test send", img)
    gst_sender_writer.write(img)
    cv2.waitKey(1)

    time.sleep(1./20)
    
    #out.write(img)
    #print("gu")
