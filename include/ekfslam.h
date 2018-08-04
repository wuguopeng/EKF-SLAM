#ifndef ekfslam_H_
#define ekfslam_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <visualization_msgs/Marker.h> //在Rviz中画点和线要用到的头文件

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h> //接收转换
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"

#include <boost/bind.hpp>

#include <cmath>
#include<iostream>  
#include<cstdlib>  
#include<ctime>

#include <Eigen/Eigen>

#define PI 3.14159265358979323846 
#define NUM 360  //激光扫描点数

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> ekfslamSyncPolicy;

struct Line
{
	double a;
	double b;

        double rho; 
	double theta;
 
        double beginx = 0;
        double endx = 0;
        int times = 1;  //线出现的次数

        int consensusNum; ////局内点个数                         
};


struct Landmark
{     	        
         double rho; 
	 double theta; 

         double a; 
	 double b; 

         double beginx = 0;
         double endx = 0;

         int id = -1; 
         int totalTimesObserved = 0; 
         int consensusNum; ////局内点个数
};


class ekfslam
{
public:
	ekfslam();
	~ekfslam();

	Eigen::Vector3d odom_robotPosition;  //由odom中读取的机器人位姿（x, y, theta），即odom坐标系下坐标
	Eigen::Vector3d robotPosition; //实际的机器人位姿(map坐标系)

	std::vector<Landmark> landmarks; //保存所有的landmarks
        std::vector<Landmark> observedLandmarks;//现在可以看到的landmarks(世界坐标系下)
	
	int numLandmarks; //landmarks的总数量
	int numObservedLandmarks;  //现在观测到的landmarksd的数量

	Eigen::MatrixXd sigma; //协方差矩阵
        Eigen::VectorXd stateX; //ekf算法的状态向量

	double linear_speed;
	double angular_speed;


       
        double coV = 0.03;
        double coV2 = coV*coV;
        double coW = PI/180;
        double coW2 = coW*coW;
        Eigen::Matrix3d Rt;  //机器人自身状态的协方差矩阵
        Eigen::Matrix2d Qt;  //测量协方差
       

private:
    const int MINOBSERVATIONS = 10; 

    const int MAXTRIALS = 100;   
    const int SAMPLEANGLE = 20; //每次采样角度范围
    const int MAXSAMPLE = 10;	  
    const int MINLINEPOINTS = 75; 
    const int CONSENSUS = 70;     
    const double TOLERANCE = 0.01;

    //设定的判断直线相似阈值
    const double rhoThreshold = 0.2;
    const double thetaThreshold = 0.3;

   //机器人扫描范围
    const double minRange = 0.2;
    const double maxRange = 3.5;      

    const double lambda = 0.01; //数据关联常数

    ros::Time current_time;
    ros::Time last_time;

    
 

public:

    void ekf(std::vector<double> ranges);
    
    void extractLandmarksFromScan(std::vector<double> ranges); //从传感器数据提取直线做为landmark,每个landmark都给一个特定的id


    void cal_stateJacobian(Eigen::VectorXd& pre_stateX, Eigen::MatrixXd& stateJacobian); ////由机器人速度和角速度，求预测的状态和计算状态方程的Jacobian矩阵

    Eigen::MatrixXd cal_outputJacobian(Landmark observedLandmark, int observedLandmark_id, Eigen::Vector3d pre_robotPosition);//计算输出方程的Jacobian矩阵

    Eigen::Vector2d MeasurementError(Landmark observedLandmark, std::vector<Line> lineRs, Eigen::Vector3d pre_robotPosition); //计算测量余差

    Eigen::Vector2d WtoR(Landmark landmark, Eigen::Vector3d robotPosition_); //Landmark是世界坐标系下的表示,坐标变换为机器人坐标系下的极坐标


    void update_sigmaAndstate(int old_numLandmarks);


public:
    void extractLineW(std::vector<double> ranges, std::vector<Line>& lines); //世界坐标系下提取的线的方程
    void extractLineR(std::vector<double> ranges, std::vector<Line>& lineRs);  //机器人坐标系下提取的线的方程


    double distanceToLine(double rho, double theta, double x, double y);   //计算点到直线的距离
    void linearLeastSquares( std::vector<Eigen::Vector2d> Points, double &a, double &b);  //Points是xy形式的坐标
    void linearTOpolar(double a, double b, double &rho, double &theta); //转化直线为极坐标形式
    bool lineCompare(Line newLine, std::vector<Line> lines); //与已经提取出的线进行对比
    double pi_limit(double angle); //保证角度在区间[-PI, PI]  
    double min(double x1, double x2);
    double max(double x1, double x2);


public:
    ros::NodeHandle nh;
   
    //用于同步订阅消息
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub;          
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_sub;  
     
    message_filters::Synchronizer<ekfslamSyncPolicy>* sync;

    void combineCallback(const sensor_msgs::LaserScanConstPtr& scanMsgAddress, const nav_msgs::Odometry::ConstPtr& odomMsgAddress);  

    void displayPL(std::vector<double> ranges);
    void publishTF(const nav_msgs::Odometry::ConstPtr& odomMsgAddress);

};


#endif
