#include "ros/ros.h"
#include <iostream>
#include <slam/Imu.h>
#include <slam/Gps.h>
#include <slam/Data.h>
#include <std_msgs/Float64MultiArray.h>
#include "Eigen/Eigen"

#define PI 3.141592653589793238463
#define SMALL 0.000001

using namespace Eigen;

template <int _ST=5, int _U=3, int _Q=3, int _ZIMU=1, int _ZGPS=2>
class Kalman_fusion{
// state : x,y,u,v,theta
// observation : x,y(,u,v,theta) from GPS, theta from IMU
//  u,v,theta can be estimated from change of GPS
// control : a_x, a_y, omega from IMU
// predict + update for each sensor input
//  predict with previous IMU control
//  update with current sensor observation
    public:
    Matrix<double,_ST,1> st;
    Matrix<double,_U,1> u;
    ros::Time t;
    Matrix<double,_ST,_ST> P;    // P is covariance of stATE
    Matrix<double,_Q,_Q> Q;
    Matrix<double,_ZIMU,_ZIMU> RIMU;
    Matrix<double,_ZGPS,_ZGPS> RGPS;
    double time_noise_var;                          
    ros::NodeHandle n_;
    ros::Publisher pub_d;
    ros::Subscriber sub_IMU;
    ros::Subscriber sub_GPS;
    int countIMU = -1;
    int countGPS = -1;

    Kalman_fusion(){
        st.setZero();
        u.setZero();
        P.setIdentity();
        Q.setIdentity();
        RIMU.setIdentity();
        RGPS.setIdentity();
        sub_IMU = n_.subscribe("/imu",1,&Kalman_fusion<>::IMUCallback,this);
        sub_GPS = n_.subscribe("/gps",1,&Kalman_fusion<>::GPSCallback,this);
        pub_d = n_.advertise<slam::Data>("filtered_data", 100);
        double initial_state_error_var, initial_a_process_noise_var, initial_omega_process_noise_var, initial_imu_observation_noise_var, initial_gps_observation_noise_var;
        ros::param::get("/initial_state_error_var", initial_state_error_var);
        ros::param::get("/initial_a_process_noise_var", initial_a_process_noise_var);
        ros::param::get("/initial_omega_process_noise_var", initial_omega_process_noise_var);
        ros::param::get("/initial_imu_observation_noise_var", initial_imu_observation_noise_var);
        ros::param::get("/initial_gps_observation_noise_var", initial_gps_observation_noise_var);
        ros::param::get("/time_noise_var", time_noise_var);
        P *= initial_state_error_var * initial_state_error_var;
        Q(0,0) *= initial_a_process_noise_var * initial_a_process_noise_var;
        Q(1,1) *= initial_a_process_noise_var * initial_a_process_noise_var;
        Q(2,2) *= initial_omega_process_noise_var * initial_omega_process_noise_var;
        RIMU *= initial_imu_observation_noise_var * initial_imu_observation_noise_var;
        RGPS *= initial_gps_observation_noise_var * initial_gps_observation_noise_var;
    }

    void IMUCallback(const slam::Imu& msg){
        if(countIMU == -1 || countGPS == -1){
            st(4) = remainder(msg.theta,2*PI);
            u(0) = msg.local_ax;
            u(1) = msg.local_ay;
            u(2) = msg.omega;
            t = msg.header.stamp;
            countIMU = 0;
            return ;
        }
        predict(msg.header.stamp);
        // z = Hst+v
        // v ~ N(0,RIMU)
        Matrix<double,_ZIMU,_ST> H;
        H << 0,0,0,0,1;
        Matrix<double,_ZIMU,1> z;
        z << msg.theta;
        Matrix<double,_ZIMU,1> y = z-H*st;
        y(0)=remainder(y(0),2*PI);

        Matrix<double,_ZIMU,_ZIMU> S = H*P*H.transpose()+RIMU;
        Matrix<double,_ST,_ZIMU> K = P*H.transpose()*S.inverse();
        P = (P.Identity()-K*H)*P;
        st = st + K*y;
        st(4)=remainder(st(4),2*PI);
        u(0)=msg.local_ax;
        u(1)=msg.local_ay;
        u(2)=msg.omega;
        publish(msg.header.stamp);
        
        countIMU+=1;
    }

    void GPSCallback(const slam::Gps& msg){
        if(countIMU == -1 || countGPS == -1){
            st(0) = msg.x;
            st(1) = msg.y;
            t = msg.header.stamp;
            countGPS = 0;
            return ;
        }
        predict(msg.header.stamp);
        // z = Hst+v
        // v ~ N(0,RGPS)
        Matrix<double,_ZGPS,_ST> H;
        H << 1,0,0,0,0 , 0,1,0,0,0;
        Matrix<double,_ZGPS,1> z;
        z << msg.x,msg.y;
        Matrix<double,_ZGPS,1> y = z-H*st;

        Matrix<double,_ZGPS,_ZGPS> S = H*P*H.transpose()+RGPS;
        Matrix<double,_ST,_ZGPS> K = P*H.transpose()*S.inverse();
        P = (P.Identity()-K*H)*P;
        st = st + K*y;
        st(4)=remainder(st(4),2*PI);
        publish(msg.header.stamp);

        countGPS+=1;
    }

    void predict(ros::Time t){
        // st = f(st,u)+w
        // F = df/dst
        // w ~ N(0,Q)
        double dt = (t-this->t).toSec();
        this->t=t;
        Matrix<double,_ST,_ST> F;
        F << 1,0,dt,0,0 , 0,1,0,dt,0 , 0,0,1,0,0 , 0,0,0,1,0 , 0,0,0,0,1;
        Matrix<double,_ST,_Q> B;
        //B << 0.5*(abs(dt)+time_noise_var)*(abs(dt)+time_noise_var),0,0 , 0,0.5*(abs(dt)+time_noise_var)*(abs(dt)+time_noise_var),0 , abs(dt)+time_noise_var,0,0 , 0,abs(dt)+time_noise_var,0 , 0,0,abs(dt)+time_noise_var;
        double dtm = abs(dt)+time_noise_var;
        double th = st(4);
        B << 0.5*dtm*dtm*abs(cos(th)),0.5*dtm*dtm*abs(sin(th)),0 , 0.5*dtm*dtm*abs(sin(th)),0.5*dtm*dtm*abs(cos(th)),0 , dtm*abs(cos(th)),dtm*abs(sin(th)),0 , dtm*abs(sin(th)),dtm*abs(cos(th)),0 , 0,0,dtm;
        st = F*st;
        F(0,4) = 0.5*dt*dt*(-sin(th)*u(0) -cos(th)*u(1)) + (1.0/6.0)*dt*dt*dt*u(2)*(-cos(th)*u(0) +sin(th)*u(1));
        F(1,4) = 0.5*dt*dt*(+cos(th)*u(0) -sin(th)*u(1)) + (1.0/6.0)*dt*dt*dt*u(2)*(-sin(th)*u(0) -cos(th)*u(1));
        F(2,4) = dt*(-sin(th)*u(0) -cos(th)*u(1)) + 0.5*dt*dt*u(2)*(-cos(th)*u(0) +sin(th)*u(1));
        F(3,4) = dt*(+cos(th)*u(0) -sin(th)*u(1)) + 0.5*dt*dt*u(2)*(-sin(th)*u(0) -cos(th)*u(1));
        P = F*P*F.transpose()+B*Q*B.transpose();
        st(0) += 0.5*dt*dt*(+cos(th)*u(0) -sin(th)*u(1)) + (1.0/6.0)*dt*dt*dt*u(2)*(-sin(th)*u(0) -cos(th)*u(1));
        st(1) += 0.5*dt*dt*(+sin(th)*u(0) +cos(th)*u(1)) + (1.0/6.0)*dt*dt*dt*u(2)*(+cos(th)*u(0) -sin(th)*u(1));
        st(2) += dt*(+cos(th)*u(0) -sin(th)*u(1)) + 0.5*dt*dt*u(2)*(-sin(th)*u(0) -cos(th)*u(1));
        st(3) += dt*(+sin(th)*u(0) +cos(th)*u(1)) + 0.5*dt*dt*u(2)*(+cos(th)*u(0) -sin(th)*u(1));
        st(4) += dt*u(2);
        st(4)=remainder(st(4),2*PI);
    }

    void publish(ros::Time t){
        slam::Data rt;
        rt.header.stamp = t;
        rt.x = st(0);
        rt.y = st(1);
        rt.theta = st(4);
        rt.omega = u(2);
        rt.local_ax = u(0);
        rt.local_ay = u(1);
        rt.v = sqrt(st(2)*st(2)+st(3)*st(3));
        rt.vx = st(2);
        rt.vy = st(3);
        pub_d.publish(rt);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman_controller");
    Kalman_fusion<> kf;
    ros::spin();
    return 0;
}
