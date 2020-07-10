#include "ros/ros.h"
#include <iostream>
#include <localization/Imu.h>
#include <localization/Gps.h>
#include <localization/Data.h>
#include <std_msgs/Float32MultiArray.h>
#include "Eigen/Eigen"

#define PI 3.141592653589793238463
#define small 0.000001

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
    Matrix<float,_U,1> u;
    ros::Time t;
    Matrix<float,_ST,_ST> P;    // P is covariance of stATE
    Matrix<float,_ST,_ST> Q;
    Matrix<float,_ZIMU,_ZIMU> RIMU;
    Matrix<float,_ZGPS,_ZGPS> RGPS;                          
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher pub_e;
    int prevType = -1;   // 0 for IMU, 1 for GPS
    Matrix<float,_ZIMU,_ZIMU> prevASIMU;
    Matrix<float,_ZGPS,_ZGPS> prevASGPS;
    Matrix<float,_ZIMU,1> prevyIMU;
    Matrix<float,_ZGPS,1> prevyGPS;
    Matrix<float,_ST,_ST> F;
    int countIMU = 0;
    int countGPS = 0;

    // R = C - HPHt
    // Q = KCKt

    Kalman_fusion(){
        st.setZero();
        u.setZero();
        P.setIdentity();
        Q.setIdentity();
        RIMU.setIdentity();
        RGPS.setIdentity();
        pub_ = n_.advertise<localization::Data>("filtered_data", 1000);
        pub_e = n_.advertise<std_msgs::Float32MultiArray>("estimated_error", 1000);
    }

    void IMUCallback(const localization::Imu& msg){
        predict(msg.header.stamp);
        // z = Hst+v ?
        // v ~ N(0,RIMU)
        Matrix<float,_ZIMU,_ST> H;
        H << 0,0,0,0,1;
        Matrix<float,_ZIMU,1> z;
        z << msg.theta;
        Matrix<float,_ZIMU,1> y = z-H*st;
        y(0)=remainderf(y(0),2*PI);


        if(prevType>-1){
            Matrix<float,_ST,_ZIMU> invH = P*H.transpose()*(H*P*H.transpose()).inverse();
            if(prevType ==0){
                updateQRIMU( invH*y*prevyIMU.transpose() );  
            }else{
                updateQRGPS( invH*y*prevyGPS.transpose() );
            }
        }

        Matrix<float,_ZIMU,_ZIMU> S = H*P*H.transpose()+RIMU;
        Matrix<float,_ST,_ZIMU> K = P*H.transpose()*S.inverse();
        P = (P.Identity()-K*H)*P;
        toSPD<_ST>(P);
        st = st + K*y;
        st(4)=remainderf(st(4),2*PI);
        u(0)=msg.local_ax;
        u(1)=msg.local_ay;
        u(2)=msg.omega;
        publish(msg.header.stamp);
        
        prevType = 0;
        prevASIMU = (S.Identity()-H*K)*y*y.transpose();
        prevyIMU = y;
        countIMU+=1;
    }
    void GPSCallback(const localization::Gps& msg){
        predict(msg.header.stamp);
        // z = Hst+v ?
        // v ~ N(0,RGPS)
        Matrix<float,_ZGPS,_ST> H;
        H << 1,0,0,0,0 , 0,1,0,0,0;
        Matrix<float,_ZGPS,1> z;
        z << msg.x,msg.y;
        Matrix<float,_ZGPS,1> y = z-H*st;

        if(prevType>-1){
            Matrix<float,_ST,_ZGPS> invH = P*H.transpose()*(H*P*H.transpose()).inverse();
            if(prevType ==0){
                updateQRIMU( invH*y*prevyIMU.transpose() );  
            }else{
                updateQRGPS( invH*y*prevyGPS.transpose() );
            }
        }

        Matrix<float,_ZGPS,_ZGPS> S = H*P*H.transpose()+RGPS;
        Matrix<float,_ST,_ZGPS> K = P*H.transpose()*S.inverse();
        P = (P.Identity()-K*H)*P;
        toSPD<_ST>(P);
        st = st + K*y;
        st(4)=remainderf(st(4),2*PI);
        publish(msg.header.stamp);

        prevType = 1;
        prevASGPS = (S.Identity()-H*K)*y*y.transpose();;
        prevyGPS = y;
        countGPS+=1;
    }

    void predict(ros::Time t){
        // st = Fst+Bu+w ?
        // w ~ N(0,Q)
        double dt = (t-this->t).toSec();
        this->t=t;
        // Matrix<float,_ST,_ST> F;
        F << 1,0,dt,0,0 , 0,1,0,dt,0 , 0,0,1,0,0 , 0,0,0,1,0 , 0,0,0,0,1;
        float th = st(4);
        st = F*st;
        F(0,4)= 0.5*dt*dt*(-sin(th)*u(0) -cos(th)*u(1));
        F(1,4)= 0.5*dt*dt*(+cos(th)*u(0) -sin(th)*u(1));
        F(2,4)= dt*(-sin(th)*u(0) -cos(th)*u(1));
        F(3,4)= dt*(+cos(th)*u(0) -sin(th)*u(1));
        P = F*P*F.transpose()+Q;
        //Matrix<float,_ST,_U> B; 
        //B << 0.5*dt*dt,0,0 , 0,0.5*dt*dt,0 , dt,0,0 , 0,dt,0 , 0,0,dt;
        //st = F*st+B*u;
        st(0) += 0.5*dt*dt*(+cos(th)*u(0) -sin(th)*u(1));
        st(1) += 0.5*dt*dt*(+sin(th)*u(0) +cos(th)*u(1));
        st(2) += dt*(+cos(th)*u(0) -sin(th)*u(1));
        st(3) += dt*(+sin(th)*u(0) +cos(th)*u(1));
        st(4)=remainderf(st(4),2*PI);
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
        pub_.publish(rt);
        
        std_msgs::Float32MultiArray rte;
        rte.data = {sqrt(P.trace()/_ST), sqrt(Q.trace()/_ST), sqrt(RIMU.trace()/_ZIMU), sqrt(RGPS.trace()/_ZGPS)};
        pub_e.publish(rte);
    }

    void updateQRIMU( Matrix<float,_ST,_ZIMU> invHS ){
        float weight = 1.0/std::min(countIMU+16,256);
        Matrix<float,_ZIMU,_ST> H;
        H << 0,0,0,0,1;
        Matrix<float,_ZIMU,_ZIMU> Rsample = prevASIMU + H*F.inverse()*invHS;
        RIMU *= 1-weight;
        RIMU += Rsample*weight;
        toSPD<_ZIMU>(RIMU);
        Matrix<float,_ST,_ST> Qsample = invHS*(H*P*H.transpose()).inverse()*H*P*F.transpose();
        Q *= 1-weight;
        Q += Qsample*weight;
        toSPD<_ST>(Q);   
    }

    void updateQRGPS( Matrix<float,_ST,_ZGPS> invHS ){
        float weight = 1.0/std::min(countGPS+16,256);
        Matrix<float,_ZGPS,_ST> H;
        H << 1,0,0,0,0 , 0,1,0,0,0;
        Matrix<float,_ZGPS,_ZGPS> Rsample = prevASGPS + H*F.inverse()*invHS;
        RGPS *= 1-weight;
        RGPS += Rsample*weight;
        toSPD<_ZGPS>(RGPS);
        Matrix<float,_ST,_ST> Qsample = invHS*(H*P*H.transpose()).inverse()*H*P*F.transpose();
        Q *= 1-weight;
        Q += Qsample*weight;
        toSPD<_ST>(Q);   
    }

    template <int dim>
    void toSPD(Matrix<float,dim,dim>& A){
        Matrix<float,dim,dim> temp = (A+A.transpose())/2;
        EigenSolver<Matrix<float,dim,dim>> es(temp);
        Matrix<float,dim,dim> D = es.pseudoEigenvalueMatrix();
        Matrix<float,dim,dim> V = es.pseudoEigenvectors();
        for(int i=0;i<dim;i+=1){
            if(D(i,i)<small){
                D(i,i)=small;
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
    //kf.P *= small;
    //kf.Q *= 0.1*0.1;
    //kf.RIMU *= 0.2*0.2;
    kf.st << -5,-10,-1,-2,-1;
    ros::Subscriber subIMU = n.subscribe("/imu",100,&Kalman_fusion<>::IMUCallback,&kf);
    ros::Subscriber subGPS = n.subscribe("/gps",100,&Kalman_fusion<>::GPSCallback,&kf);
    ros::spin();
    return 0;
}