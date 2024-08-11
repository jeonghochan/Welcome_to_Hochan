#전반적인 구조

#functions for run : fmtc_remote-ar_hud/util/camera_viz.py
#run file : start.sh #이안에 이제 디렉토리와 new_ui.py를 실행할 컴파일된 파일 디렉토리 존재
#/home/hochan/gitpractice-main/fmtc_remote-ar_hud/util/network_eval.ipynb 여기에 이제 신호 통신 딜레이 결과 파일

#run file은 new_ui.py or temp_ui.py









#readme 해석\
FMTC Remote 프로젝트 설정 지침
1. 햅틱 장치 제어 파일 컴파일

프로젝트에는 G29 포스 피드백 장치를 제어하는 코드를 포함하고 있습니다. 이 코드는 C++로 작성되어 있으며, Python에서 사용하기 위해 공유 라이브러리(.so 파일)로 컴파일해야 합니다.
C++ 파일 컴파일

    메인 파일: g29_force_feedback.cpp는 주로 토크를 계산하고 제어하는 역할을 합니다.
    공유 라이브러리 생성: 다음 명령어를 터미널에서 실행하여 .so 파일을 생성합니다.

bash

g++ -c -fPIC g29_force_feedback.cpp -o g29_lib_shinkansan.o
g++ -shared g29_lib_shinkansan.o -o g29_lib_shinkansan.so

    -c: 소스 파일을 개체 파일(.o)로 컴파일합니다.
    -fPIC: 주소 독립적 코드를 생성하여 공유 라이브러리를 지원합니다.
    -shared: 공유 라이브러리 파일(.so)을 생성합니다.

Python에서 공유 라이브러리 사용

    g29_adapter.py: 이 파일은 생성한 .so 파일을 불러와 Python 클래스 형태로 포장합니다.
    사용 예시: adapter.py를 참고하여 공유 라이브러리를 사용하는 방법을 배울 수 있습니다.

2. 요구사항 설치

프로젝트에서 필요한 패키지를 설치해야 합니다.
Python 패키지 설치

    requirements.txt 파일에 정의된 Python 패키지를 설치합니다.

bash

pip install -r requirements.txt

OpenCV 설치 및 GStreamer와의 통합

OpenCV를 GStreamer와 함께 사용할 수 있도록 컴파일해야 합니다.

    GStreamer 및 개발 파일 설치:

    bash

sudo apt-get update
sudo apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav

OpenCV 컴파일:

CMake를 사용하여 OpenCV를 GStreamer와 함께 컴파일합니다. 아래 명령어를 사용합니다:

bash

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

이 명령어는 OpenCV를 설치 디렉토리에 Python과 함께 설치하며, GStreamer를 지원하도록 설정합니다.

컴파일 및 설치:

bash

    make -j$(nproc)  # 병렬 컴파일
    sudo make install
    sudo ldconfig  # 라이브러리 캐시 업데이트

3. ROS 및 Carla 지원을 위한 패키지 설치

    rospkg와 catkin_pkg 설치: 이 두 패키지는 ROS와 통합된 환경에서 작업할 때 필요합니다.

bash

pip install rospkg catkin_pkg

요약

    C++ 파일을 컴파일하여 Python에서 사용할 수 있는 공유 라이브러리를 생성합니다.
    필요한 Python 패키지를 설치하고 OpenCV를 GStreamer와 통합하여 컴파일합니다.
    ROS와 Carla 지원을 위해 rospkg와 catkin_pkg를 설치합니다.