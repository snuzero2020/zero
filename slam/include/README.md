* UTM.h
  * int LatLonToUTMXY (FLOAT lat, FLOAT lon, int zone, FLOAT& x, FLOAT& y)    
위경도(lat, lon)를 UTM52N 좌표(x, y)로 변환하는 함수. GPS로부터 받은 위치 정보를 변환할 때 사용한다.

* UnixtimeToSec.h
  * int UnixtimeToSec(int unixtime)    
[유닉스 시간](https://ko.wikipedia.org/wiki/%EC%9C%A0%EB%8B%89%EC%8A%A4_%EC%8B%9C%EA%B0%84)을 0~86400초 사이의 값으로 변환하는 함수.

* XYToPixel.h
  * void XYToPixel(int& pixel_x, int& pixel_y, double x, double y, bool is_kcity)    
UTM52N 좌표(x, y)를 지도 픽셀 값(pixel_x, pixel_y)로 변환하는 함수.

  * void PixelToXY(double& x, double& y, int pixel_x, int pixel_y, bool is_kcity)    
지도 픽셀 값(pixel_x, pixel_y)을 UTM52N 좌표(x, y)로 변환하는 함수.
