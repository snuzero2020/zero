#include "ros/ros.h"
#include <iostream>
#include <localization/Imu.h>
#include <localization/Gps.h>
#include <localization/Data.h>
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
    ros::NodeHandle n_;
    ros::Publisher pub_d;
    ros::Publisher pub_e;
    ros::Publisher pub_c;
    int prevType = -1;   // 0 for IMU, 1 for GPS
    Matrix<double,_ZIMU,_ZIMU> prevASIMU;
    Matrix<double,_ZGPS,_ZGPS> prevASGPS;
    Matrix<double,_ZIMU,1> prevyIMU;
    Matrix<double,_ZGPS,1> prevyGPS;
    Matrix<double,_ST,_ST> F;
    Matrix<double,_ST,_Q> B;
    Matrix<double,_ST,_Q> prevB;
    int countIMU = -1;
    int countGPS = -1;

    Kalman_fusion(){
        st.setZero();
        u.setZero();
        P.setIdentity();
        Q.setIdentity();
        RIMU.setIdentity();
        RGPS.setIdentity();
        pub_d = n_.advertise<localization::Data>("filtered_data", 10);
        pub_e = n_.advertise<std_msgs::Float64MultiArray>("estimated_err", 10);
        pub_c = n_.advertise<std_msgs::Float64MultiArray>("estimated_cov", 10);
    }

    void IMUCallback(const localization::Imu& msg){
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

        if(prevType>-1){
            Matrix<double,_ST,_ZIMU> invH = P*H.transpose()*(H*P*H.transpose()).inverse();
            if(prevType ==0){
                updateQRIMU( invH*y*prevyIMU.transpose() );  
            }else{
                updateQRGPS( invH*y*prevyGPS.transpose() );
            }
        }

        Matrix<double,_ZIMU,_ZIMU> S = H*P*H.transpose()+RIMU;
        Matrix<double,_ST,_ZIMU> K = P*H.transpose()*S.inverse();
        P = (P.Identity()-K*H)*P;
        toSPD<_ST>(P);
        st = st + K*y;
        st(4)=remainder(st(4),2*PI);
        u(0)=msg.local_ax;
        u(1)=msg.local_ay;
        u(2)=msg.omega;
        publish(msg.header.stamp);
        
        prevType = 0;
        prevASIMU = (S.Identity()-H*K)*y*y.transpose();
        prevyIMU = y;
        countIMU+=1;
        prevB=B;
    }

    void GPSCallback(const localization::Gps& msg){
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

        if(prevType>-1){
            Matrix<double,_ST,_ZGPS> invH = P*H.transpose()*(H*P*H.transpose()).inverse();
            if(prevType ==0){
                updateQRIMU( invH*y*prevyIMU.transpose() );  
            }else{
                updateQRGPS( invH*y*prevyGPS.transpose() );
            }
        }

        Matrix<double,_ZGPS,_ZGPS> S = H*P*H.transpose()+RGPS;
        Matrix<double,_ST,_ZGPS> K = P*H.transpose()*S.inverse();
        P = (P.Identity()-K*H)*P;
        toSPD<_ST>(P);
        st = st + K*y;
        st(4)=remainder(st(4),2*PI);
        publish(msg.header.stamp);

        prevType = 1;
        prevASGPS = (S.Identity()-H*K)*y*y.transpose();
        prevyGPS = y;
        countGPS+=1;
        prevB=B;
    }

    void predict(ros::Time t){
        // st = f(st,u)+w
        // F = df/dst
        // w ~ N(0,Q)
        double dt = (t-this->t).toSec();
        this->t=t;
        // Matrix<double,_ST,_ST> F;
        F << 1,0,dt,0,0 , 0,1,0,dt,0 , 0,0,1,0,0 , 0,0,0,1,0 , 0,0,0,0,1;
        B << 0.5*dt*dt,0,0 , 0,0.5*dt*dt,0 , dt,0,0 , 0,dt,0 , 0,0,dt;
        double th = st(4);
        st = F*st;
        F(0,4)= 0.5*dt*dt*(-sin(th)*u(0) -cos(th)*u(1));
        F(1,4)= 0.5*dt*dt*(+cos(th)*u(0) -sin(th)*u(1));
        F(2,4)= dt*(-sin(th)*u(0) -cos(th)*u(1));
        F(3,4)= dt*(+cos(th)*u(0) -sin(th)*u(1));
        P = F*P*F.transpose()+B*Q*B.transpose();
        st(0) += 0.5*dt*dt*(+cos(th)*u(0) -sin(th)*u(1));
        st(1) += 0.5*dt*dt*(+sin(th)*u(0) +cos(th)*u(1));
        st(2) += dt*(+cos(th)*u(0) -sin(th)*u(1));
        st(3) += dt*(+sin(th)*u(0) +cos(th)*u(1));
        st(4) += dt*u(2);
        st(4)=remainder(st(4),2*PI);
    }

    void publish(ros::Time t){
        localization::Data rt;
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
        
        std_msgs::Float64MultiArray rte;
        rte.data = {sqrt(P(0,0)),sqrt(P(1,1)),sqrt(P(2,2)),sqrt(P(3,3)),sqrt(P(4,4))};
        pub_e.publish(rte);

        std_msgs::Float64MultiArray rtc;
        rtc.data = {sqrt(Q.trace()/_Q), sqrt(RIMU.trace()/_ZIMU), sqrt(RGPS.trace()/_ZGPS)};
        pub_c.publish(rtc);
    }

    void updateQRIMU( Matrix<double,_ST,_ZIMU> invHS ){
        double weight = 1.0/std::min(countIMU+64,1024);
        Matrix<double,_ZIMU,_ST> H;
        H << 0,0,0,0,1;
        Matrix<double,_ZIMU,_ZIMU> Rsample = prevASIMU + H*F.inverse()*invHS;
        RIMU *= 1-weight;
        RIMU += Rsample*weight;
        toSPD<_ZIMU>(RIMU);
        Matrix<double,_Q,_ST> invB = (prevB.transpose()*prevB).completeOrthogonalDecomposition().pseudoInverse()*prevB.transpose();
        Matrix<double,_Q,_Q> Qsample = invB * invHS*(H*P*H.transpose()).inverse()*H*P*F.transpose() * invB.transpose();
        Q *= 1-weight;
        Q += Qsample*weight;
        toSPD<_Q>(Q);
    }

    void updateQRGPS( Matrix<double,_ST,_ZGPS> invHS ){
        double weight = 1.0/std::min(countGPS+64,1024);
        Matrix<double,_ZGPS,_ST> H;
        H << 1,0,0,0,0 , 0,1,0,0,0;
        Matrix<double,_ZGPS,_ZGPS> Rsample = prevASGPS + H*F.inverse()*invHS;
        RGPS *= 1-weight;
        RGPS += Rsample*weight;
        toSPD<_ZGPS>(RGPS);
        Matrix<double,_Q,_ST> invB = (prevB.transpose()*prevB).completeOrthogonalDecomposition().pseudoInverse()*prevB.transpose();
        Matrix<double,_Q,_Q> Qsample = invB * invHS*(H*P*H.transpose()).inverse()*H*P*F.transpose() * invB.transpose();
        Q *= 1-weight;
        Q += Qsample*weight;
        toSPD<_Q>(Q);
    }

    template <int dim>
    void toSPD(Matrix<double,dim,dim>& A){
        Matrix<double,dim,dim> temp = (A+A.transpose())/2;
        EigenSolver<Matrix<double,dim,dim>> es(temp);
        Matrix<double,dim,dim> D = es.pseudoEigenvalueMatrix();
        Matrix<double,dim,dim> V = es.pseudoEigenvectors();
        for(int i=0;i<dim;i+=1){
            if(D(i,i)<SMALL){
                D(i,i)=SMALL;
            }
        }
        A = V*D*V.transpose();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman_controller");
    ros::NodeHandle n;

    Kalman_fusion<> kf;
    kf.P *= 10*10;
    kf.Q *= 400*400;
    kf.RIMU *= 0.005*0.005;
    kf.RGPS *= 0.1*0.1;
    
    ros::Subscriber subIMU = n.subscribe("/imu",100,&Kalman_fusion<>::IMUCallback,&kf);
    ros::Subscriber subGPS = n.subscribe("/gps",100,&Kalman_fusion<>::GPSCallback,&kf);
    ros::spin();
    return 0;
}