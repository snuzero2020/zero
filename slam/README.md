# SNUZERO SLAM team
This is SLAM team repository!

# Clone

```
git clone https://github.com/ros-drivers/nmea_msgs.git ~/catkin_ws/src/nmea_msgs
sudo apt-get install python-pyproj
```

# Repository
## Messages
* Cluster
* Data
* Gps
* Imu
* Lidar

## Header Files
* UTM   
Convert a WGS84(EPSG::4326) coordinate to a UTM52(EPSG::32652) coordinate.   

  - int LatLonToUTMXY (double lat, double lon, int zone, double& x, double& y)   
  Convert a received WGS84 coordinate(lat, lon) to a UTM${zone} coordinate(x, y). For Korea, let zone 52.   
  
  - Input   
    + double lat, double lon: The car's position.   
    + int zone: the zone at which the place, on which the car drive, is located.   
    + double& x, double& y: The car's converted position.   
  
  - Return   
    + 0: No error.   
    + -1: One or more errors occured.   

*****

* UnixtimeToSec   
Convert the received Unix time to a count starting from A.M. 12:00. to just before next day A.M. 12:00.   
e.g. 1595580968(Unix time) → Fri Jul 24 2020 17:56:08 UTC+0900 → (17 * 3600) + (56 * 60) + 8 = 64568   

  - int UnixtimeToSec(int unixtime)
  Convert the received Unix time to a count.   
  
  - Input   
    + int unixtime: The unix time to converted.   

  - Return   
    + The converted time(by sec).   

*****

* XYToPixel   
Convert a UTM52 coordinate to a position(pixel) on the received map.   

  - int XYToPixel(cv::Mat img, double x, double y, int& pixel_x, int& pixel_y, int place)
    Convert a received UTM52 coordinate(x, y) to a position(pixel) on the received map(pixel_x, pixel_y).   
    
  - Input   
    + Mat img: The map on which the car drive.   
    + double x, double y: The car's position in UTM52 coordinate(x, y).   
    + int pixel_x, int pixel_y: The car's converted position(pixel_x, pixel_y).   
    + int place:   
        1: K-City   
        2: FMTC   

  - Return   
    + 0: No error.   
    + -1: One or more errors occured.   

## Source Codes
* BGRmap_to_costmap
* clustering
* localmap_pubnode
* mapping
* sensing
