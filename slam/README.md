### 목차
- [디렉토리 구조](#디렉토리-구조)
- [Driver 설치](#driver-설치)
  * [GPS](#gps)
  * [IMU](#imu)
  * [기타 패키지](#기타-패키지)
- [Driver 사용](#driver-사용)
  * [GPS](#gps-1)
  * [IMU](#imu-1)
- [IMU Configuration](#imu-configuration)
  * [MTManager](#mtmanager)
  * [Calibration](#calibration)

# 디렉토리 구조
* config/   
FMTC와 K-City에서 활용하는 지도(global map, costmap, velocity map, ...)와 경로(global path)
* include/   
lib에 있는 함수들을 사용할 때 필요한 헤더 파일
* launch/   
여러 노드를 한번에 실행하기 위한 런치 파일 
* lib/   
여러 소스 코드에서 쓰이는 함수들
* msg/   
ROS 내부 통신에서 사용하는 커스텀 포맷(클러스터, YOLO 정보, 주차 영역, ...)
* src/   
소스 코드
* CMakeLists*.txt   
Cmake을 이용한 컴파일에 필요한 옵션을 제공하는 파일
* ~~KMTC.yaml~~   
사용하지 않음
* slam_para.yaml   
SLAM의 각 노드와 프로그램에서 필요로 하는 매개 변수(parameter)를 저장하는 파일
* package.xml   
ROS에서 각 노드의 존재와 관계를 알려주는 파일

-----

# Driver 설치
## GPS
```
sudo apt-get install ros-melodic-nmea-msgs
sudo apt-get install ros-melodic-nmea-navsat-driver
```

## IMU
http://wiki.ros.org/xsens_mti_driver 참고   
https://www.xsens.com/mt-software-suite/ 에서 왼쪽 위 MTi Products의 Latest Stable Software 다운   
다운 받은 압축파일을 압축 푼 다음 터미널에서 그 디렉토리로 이동해서   
```
sudo apt-get install sharutills
sudo sh mtsdk_linux-x64_2019.2.sh
```
하면 Enter the installation directory [/usr/local/xsens] > 에   
##### /usr/local/xsens   
한 다음 /usr/local/xsens에서 xsens_ros_mti_driver 찾아서 catkin_ws/src에 복사   
catkin_ws로 이동해서   
```
pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
catkin_make
```

## 기타 패키지
```
sudo apt-get install python-pyproj
```

# Driver 사용
```
roslaunhch slam sensor_driver.launch
```
각자 켜려면 아래 참고

## GPS
(roscore 혹은 roslaunch 실행 필요)   
```
sudo chmod 777 /dev/포트이름
rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/포트이름 _baud:=115200
```

## IMU
```sudo chmod 777 /dev/포트이름
roslaunch xsens_mti_driver xsens_mti_node.launch
```

# IMU Configuration
## MTManager
[https://www.xsens.com/mt-software-suite/](https://www.xsens.com/mt-software-suite/)에서 받은 압축파일을 압축 푼 다음   
그 안에서 mtmanager를 찾아 압축 해제   
압축 푼 mtmanager 디렉토리 /linux-x64/doc/MTM.README 참고   
```
sudo apt-get install libqt5opengl5   
sudo apt-get install libusb-1.0-0   
sudo apt-get install libxcb-xinerama0   
```
그 다음 [https://packages.ubuntu.com/xenial/amd64/libpng12-0/download](https://packages.ubuntu.com/xenial/amd64/libpng12-0/download)에서 다운받아 설치   
   
```
sudo chmod 777 /dev/포트이름
```
압축 푼 mtmanager 디렉토리 /linux-x64/bin 에서   
```
./mtmanager
```

## Calibration
우분투에서는 버전 문제로 잘 안 되어서 Windows 사용   
설명 동영상: https://tutorial.xsens.com/video/magnetic-calibration   

https://www.xsens.com/software-downloads 에서 Download MT Software Suite 클릭하고 윈도우용 다운   
다운받은 exe파일을 실행하여 common과 magnetic field mapper 설치   
imu를 연결하고 magnetic field mapper 실행   
지시사항대로 한 다음   
(새 데이터 사용)Use Motion Tracker 클릭하고 Scan   
스캔된 imu 클릭하고 다음 누르다가 측정 시작   
차를 다양한 방향으로 돌림   
측정 끝 누름   
결과가 나오고 write 누르면 기록 됐다는 창이 뜸   
