#include "ros/ros.h"
#include <iostream>
#include <sensor_fusion/Imu.h>
#include <sensor_fusion/Gps.h>
#include "Eigen/Eigen"

using namespace Eigen;

template <int _ST=5, int _U=3, int _ZIMU=1, int _ZGPS=2>
class Kalman_fusion{
// state : x,y,u,v,theta
// observation : x,y(,u,v,theta) from GPS, theta from IMU
//  u,v,theta can be estimated from change of GPS
// control : a_x, a_y, omega from IMU
// predict + update for each sensor input
//  predict with previous IMU control
//  update with current sensor observation
    public:
    Matrix<float,_ST,1> st;
    Matrix<float,_ST,_ST> P;
    Matrix<float,_U,1> u;
    ros::Time t;
    Matrix<float,_ST,_ST> Q;
    Matrix<float,_ZIMU,_ZIMU> RIMU;
    Matrix<float,_ZGPS,_ZGPS> RGPS;
    
    Kalman_fusion(){
        st.setZero();
        P.setIdentity();
        u.setZero();
        Q.setIdentity();
        RIMU.setIdentity();
        RGPS.setIdentity();
    }

    Kalman_fusion(Matrix<float,_ST,1> st, Matrix<float,_ST,_ST> P, Matrix<float,_U,1> u, Matrix<float,_ST,_ST> Q, Matrix<float,_ZIMU,_ZIMU> RIMU, Matrix<float,_ZGPS,_ZGPS> RGPS){
        this->st=st;
        this->P=P;
        this->u=u;
        this->Q=Q;
        this->RIMU=RIMU;
        this->RGPS=RGPS;
    }

    void IMUCallback(const sensor_fusion::Imu& msg){
        predict(msg.header.stamp);
        // z = Hst+v ?
        // v ~ N(0,RIMU)
        Matrix<float,_ZIMU,_ST> H;
        H << 0,0,0,0,1;
        Matrix<float,_ZIMU,1> z;
        z << msg.th;
        Matrix<float,_ZIMU,_ZIMU> S = H*P*H.transpose()+RIMU;
        Matrix<float,_ST,_ZIMU> K = P*H.transpose()*S.inverse();
        P = (P.Identity()-K*H)*P;
        st = st + K*(z - H*st);

        u(0)=msg.ax;
        u(1)=msg.ay;
        u(2)=msg.om;
    }
    void GPSCallback(const sensor_fusion::Gps& msg){
        predict(msg.header.stamp);
        // z = Hst+v ?
        // v ~ N(0,RGPS)
        Matrix<float,_ZGPS,_ST> H;
        H << 1,0,0,0,0 , 0,1,0,0,0;
        Matrix<float,_ZGPS,1> z;
        z << msg.x,msg.y;
        Matrix<float,_ZGPS,_ZGPS> S = H*P*H.transpose()+RGPS;
        Matrix<float,_ST,_ZGPS> K = P*H.transpose()*S.inverse();
        P = (P.Identity()-K*H)*P;
        st = st + K*(z - H*st);
    }

    void predict(ros::Time t){
        // st = Fst+Bu+w ?
        // w ~ N(0,Q)
        double dt = (t-this->t).toSec();
        this->t=t;
        Matrix<float,_ST,_ST> F;
        F << 1,0,dt,0,0 , 0,1,0,dt,0 , 0,0,1,0,0 , 0,0,0,1,0 , 0,0,0,0,1;
        float th = st(2);
        st = F*st;
        F(0,4)= 0.5*dt*dt*(-sin(th)*u(0) -cos(th)*u(1)) + 1/6*dt*dt*dt*(-cos(th)*u(0) +sin(th)*u(1))*u(2);
        F(1,4)= 0.5*dt*dt*(+cos(th)*u(0) -sin(th)*u(1)) + 1/6*dt*dt*dt*(-sin(th)*u(0) -cos(th)*u(1))*u(2);
        F(2,4)= dt*(-sin(th)*u(0) -cos(th)*u(1)) + 0.5*dt*dt*(-cos(th)*u(0) +sin(th)*u(1))*u(2);
        F(3,4)= dt*(+cos(th)*u(0) -sin(th)*u(1)) + 0.5*dt*dt*(-sin(th)*u(0) -cos(th)*u(1))*u(2);
        P = F*P*F.transpose()+Q;
        //Matrix<float,_ST,_U> B; 
        //B << 0.5*dt*dt,0,0 , 0,0.5*dt*dt,0 , dt,0,0 , 0,dt,0 , 0,0,dt;
        //st = F*st+B*u;
        st(0) += 0.5*dt*dt*(+cos(th)*u(0) -sin(th)*u(1)) + 1/6*dt*dt*dt*(-sin(th)*u(0) -cos(th)*u(1))*u(2);
        st(1) += 0.5*dt*dt*(+sin(th)*u(0) +cos(th)*u(1)) + 1/6*dt*dt*dt*(+cos(th)*u(0) -sin(th)*u(1))*u(2);
        st(2) += dt*(+cos(th)*u(0) -sin(th)*u(1)) + 0.5*dt*dt*(-sin(th)*u(0) -cos(th)*u(1))*u(2);
        st(3) += dt*(+sin(th)*u(0) +cos(th)*u(1)) + 0.5*dt*dt*(+cos(th)*u(0) -sin(th)*u(1))*u(2);
    }
};

template <int _ST=5, int _U=3, int _ZIMU=1, int _ZGPS=4>
class Kalman_fusion_with_GPS_change : public Kalman_fusion<_ST,_U,_ZIMU,_ZGPS>{
    //  u,v,theta can be estimated from change of GPS
    Matrix<float,_ZIMU,_ZIMU> RIMU;
    double prevZx;
    double prevZy;
    ros::Time prevZt;
    bool isFirst=true;
    using Kalman_fusion<_ST,_U,_ZIMU,_ZGPS>::st;
    using Kalman_fusion<_ST,_U,_ZIMU,_ZGPS>::P;
    using Kalman_fusion<_ST,_U,_ZIMU,_ZGPS>::RGPS;
    using Kalman_fusion<_ST,_U,_ZIMU,_ZGPS>::predict;

    void GPSCallback(const sensor_fusion::Gps& msg){
    
        if(isFirst==true){
            isFirst=false;
        }else{
            predict(msg.header.stamp);
            // z = Hst+v
            // v ~ N(0,RGPS)
            Matrix<float,_ZGPS,_ST> H;
            H << 1,0,0,0,0 , 0,1,0,0,0 , 0,0,1,0,0 , 0,0,0,1,0;
            double dt = (msg.header.stamp-prevZt).toSec();
            Matrix<float,_ZGPS,1> z;
            z << msg.x,msg.y,(msg.x-prevZx)/dt,(msg.y-prevZy)/dt;
            Matrix<float,_ZGPS,_ZGPS> S = H*P*H.transpose()+RGPS;
            Matrix<float,_ST,_ZGPS> K = P*H.transpose()*S.inverse();
            P = (P.Identity()-K*H)*P;
            st = st + K*(z - H*st);
        }
        prevZx = msg.x;
        prevZy = msg.y;
        prevZt = msg.header.stamp;           
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);

    Kalman_fusion<> kf;
    ros::Subscriber subIMU = n.subscribe("/imu",100,&Kalman_fusion<>::IMUCallback,&kf);
    ros::Subscriber subGPS = n.subscribe("/gps",100,&Kalman_fusion<>::GPSCallback,&kf);
    ros::spin();
    return 0;
}
