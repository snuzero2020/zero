#include "ros/ros.h"
#include "ros/time.h"
#include <ncurses.h>
#include <iostream>
#include "localization/FloatStamp.h"

int main(int argc, char **argv){
    initscr();    
    printw("Hello world");  
    refresh();    
    getch();    
    endwin();
    ros::init(argc, argv, "keyop");
    ros::NodeHandle n;
    ros::Publisher keyop_pub = n.advertise<localization::FloatStamp>("keyop",1000);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        localization::FloatStamp msg;
        float x;
        std::cin >> x;
        msg.data = x;
        keyop_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}