#include <cmath>

int UnixtimeToSec(int unixtime) {
    int sec = unixtime % 60;
    int min = ((unixtime - sec) / 60) % 60;
    int hour = ((((unixtime - sec) / 60) - min) / 60) % 24;
    return 3600*hour + 60*min + sec;
}