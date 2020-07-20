#!/usr/bin/env python

import rospy
from nmea_msgs.msg import Sentence
from localization.msg import Gps
from pyproj import Proj, transform
#from functools import reduce ## for python3 only

class GPS_Decoder:
    proj_WGS84 = Proj(init = 'epsg:4326')
    proj_5186 = Proj(init = 'epsg:5186') # for FMTC
    proj_UTMK = Proj(init = 'epsg:5178') # for K-city
    proj_UTM52N = Proj(init = 'epsg:32652')
        
    def __init__(self):
        self.pub_ = rospy.Publisher("/gps", Gps, queue_size=10)
        self.sub_ = rospy.Subscriber("/nmea_sentence",Sentence, self.callback)

    def callback(self, msg):
        rt = Gps()
        s = msg.sentence
        tokens = s.split(',')
        if tokens[0] == "$GNGGA":
            for token in tokens[1:9]:
                if token == "":
                    return
            if int(tokens[6])<2:
                rospy.logerr("failedByQualityIndicator(No-Fix), GPS Quality Indicator: %s", tokens[6])
                return
            if int(tokens[6])<4:
                rospy.logwarn("warnByQualityIndicator(No-RTK), GPS Quality Indicator: %s", tokens[6])
            if int(tokens[7])<3:
                rospy.logerr("failedByStatellitesN, The number of Satellite: %s", tokens[7])
                return
            if int(tokens[7])<8:
                rospy.logwarn("warnByStatellitesN, The number of Satellite: %s", tokens[7])
            if (float(tokens[8])>7.5 or float(tokens[8])==0.0):
                rospy.logerr("failedByHDOP, HDOP value: %s", tokens[8])
                return
            if float(tokens[8])>3.5:
                rospy.logwarn("warnByHDOP, HDOP value: %s", tokens[8])
            
            checksum = s[1:].split('*')
            cs0 = reduce(lambda x,y:x^y, map(ord,checksum[0]))
            cs1 = int(checksum[1],16)
            if cs0 != cs1:
                rospy.logerr("FailedByChecksum, %d != %d == 0x%s", cs0, cs1, checksum[1])
                return

            # hour = int(tokens[1][0:2])
            # minu = int(tokens[1][2:4])
            # sec = float(tokens[1][4:])
            # time = hour*3600 + minu*60 + sec
            lat = int(tokens[2][0:2]) + float(tokens[2][2:])/60.0
            lon = int(tokens[4][0:3]) + float(tokens[4][3:])/60.0
            
            rt.header = msg.header
            rt.x, rt.y = transform(self.proj_WGS84,self.proj_5186,lon,lat)
            #rt.header.stamp = rospy.Time(time)
            #rt.header.stamp = rospy.get_rostime()
            rt.err_identifier = float(tokens[6])
            rt.err_hdop = float(tokens[8])
            self.pub_.publish(rt)
        
    

if __name__ == '__main__':
    rospy.init_node('gps_decoder')
    gps_decoder=GPS_Decoder()
    rospy.spin()
