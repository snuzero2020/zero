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
Convert a colored map(BGR map) to a cost map(grayscale map). costs should have a value from 0 to 100.   

  - static double getDistance(Point point1, Point point2)   
    Calculate the (Euclidian) distance between received two points.   
    
  - Input   
    + Point point1, Point point2: Two points between which calculate the distance.   
    
  - Return   
    The Euclidian distance between received two points.   
*****
  - static int getDistanceInt(Point point1, Point point2)   
    Calculate the (Euclidian) distance between received two points, and translate it into the intager value.   
        
  - Input   
    + Point point1, Point point2: Two points between which calculate the distance.   
        
  - Return   
    The integer Euclidian distance between received two points.   
  *****
  - static void calculateCost(int row_start, int row_end, int id)   
    Calculate a bounding box on which cost will be distributed, and distribute costs based on the static member variables(lambda function).   
  
  - Input   
    + int row_start, int row_end: Determine where calculation starts from the row, and ends.   
    + int id: ID of the thread that perform the calcutation.   
  *****
  - BGRmapToCostmap(Mat p_BGRmap)   
    Constructor with a received colored(BGR) map.   
    By using this constructor, you can mount the colored map you want.   
        
  - Input   
    + Mat p_BGRmap: The colored map you want.   
*****
  - void setBGRmap(Mat p_BGRmap)   
    Function with a received colored(BGR) map.   
    If you use the constructor without any parameter, then you should this function so that mounting a colored map.   
        
  - Input   
    + Mat p_BGRmap: The colored map you want.   
*****
  - Mat getCostmap()   
    Retrive the costmap from the object you created.   
        
  - Return   
    The costmap from the object.   
*****
  - void transform(function<double(uchar, uchar, uchar)>& weight, function<int(int)>& formula, int p_scope, double p_threshold, int core)    
    Transform the BGR image as the object's member variable into the costmap image(grayscale), using the weight lambda function and foumula(e.g. 1/r^2) lambda function passed.   
        
  - Input   
    + function<double(uchar, uchar, uchar)>& weight: The weight applied to this transfomation.   
    + function<int(int)>& formula: The formula applied to this transfomation(cost distribution). This formula is like 1/r^2 or 1/arctan(r).   
                                   Assume this parameter is '1/r', for example, then a cost will be distributed propotionally '1/r'(r is distance) from the center(cost_seed).   
    + int p_scope: Determine how much far cost will be distributed.   
                   If this parameter is negative, then 'p_scope' is ignored, but 'p_threshold' become important.   
                   If this parameter is not negative, then the cost will be distributed to the distance from the center, this 'p_scope'.   
    + double p_threshold: This parameter is applied only when 'p_scope' is negative.   
                          if 'p_threshold' < formula(distance) * 100, then minimum value of distance that satisfy the left condition become 'p_scope', and 'scope'(private member variable).   
     
         note: 100 is maximum value for the costs.   
    + int core: Determine hou much threads will be created to execute this transformation.   
  *****
* clustering
* localmap_pubnode
* mapping
* sensing
