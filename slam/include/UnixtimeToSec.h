int UnixtimeToSec(int unixtime);
// Convert the received Unix time to a count starting from A.M. 12:00. to just before next day A.M. 12:00.
//
// Input
//     int unixtime: The unix time to converted.
// 
// Return
//     The converted time(by sec).