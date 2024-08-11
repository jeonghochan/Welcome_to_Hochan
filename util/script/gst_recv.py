import cv2
import matplotlib.pyplot as plt

testgst=cv2.VideoCapture('udpsrc port=5118 \
    ! application/x-rtp,width=3840,height=720,encoding-name=H264,framerate=27/1,payload=96  \
    ! rtpjitterbuffer latency=0  \
    ! rtph264depay  \
    ! h264parse \
    ! avdec_h264 \
    ! videoconvert \
    ! appsink', cv2.CAP_GSTREAMER)

while True:

    ret, frameRaspiIP=testgst.read()

    if ret == False: continue
    
    img = cv2.resize(frameRaspiIP, dsize=None, fx=0.5, fy=0.5)

    cv2.imshow('test recv',img)

    if cv2.waitKey(1)==ord('q'):
        break

testgst.release()

cv2.destroyAllWindows()