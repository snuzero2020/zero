/**
@brief 유닉스 시간(Unix time)을 0~86399 사이의 초 값으로 변환한다.

@param unixtime 변환할 유닉스 시간.
@return 변환된 초 값. 0부터 86399 사이의 값을 갖는다.
*/
int UnixtimeToSec(int unixtime);
// Convert the received Unix time to a count starting from A.M. 12:00. to just before next day A.M. 12:00.
//
// Input
//     int unixtime: The unix time to converted.
// 
// Return
//     The converted time(by sec).