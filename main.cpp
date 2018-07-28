#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include<geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h> //接收转换
#include <visualization_msgs/Marker.h> //在Rviz中画点和线要用到的头文件

#include <boost/bind.hpp>

#include "extractLines.h"


#define NUM 360

using std::string;
using std::cout;
using std::endl;


void scanCallback(const sensor_msgs::LaserScan::ConstPtr scanMsgAddress, const tf::TransformListener& listener, std::vector<Line>& lines);

int main(int argc,char ** argv)
{
     ros::init(argc,argv,"extractlines");
     ros::NodeHandle n;
     
     tf::TransformListener listener;  //创建一个监听对象
     std::vector<Line> lines;

     //利用boost::bind()可以给回调函数传递多个参数, _1是占位符. 而消息的前面因为是_1占位符传入，所以回调函数里参数调用的msg前不能加&
     ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>( "/scan", 10, boost::bind(&scanCallback, _1, boost::ref(listener), lines) );
     //cout<<lines.size()<<endl;   
     ros::spin();  
     return 0;
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr scanMsgAddress, const tf::TransformListener& listener, std::vector<Line>& lines)
{
    ros::NodeHandle nh;
    std::vector<double> ranges;
    double z = 0;
    
     //tf::StampedTransform transform;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
 
    //画扫描的点和提取的直线
      visualization_msgs::Marker points, line_list;
     
      
      points.header.frame_id = "odom";
      points.header.stamp = ros::Time::now();
      points.ns = "points";
      points.action = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = 1.0;
      points.id = 0;
      points.type = visualization_msgs::Marker::POINTS;
      // scale成员对于这些marker类型是不同的,POINTS marker分别使用x和y作为宽和高
      points.scale.x = 0.02;
      points.scale.y = 0.02;
      // 点为红色
      points.color.r = 1.0f;
      points.color.a = 1.0;

      for(int i=0; i<NUM; i++)
      {
           ranges.push_back(scanMsgAddress->ranges[i]);
           geometry_msgs::Point scan_point;

           geometry_msgs::PointStamped scan_point2;
           scan_point2.header.stamp=ros::Time(0);
           scan_point2.header.frame_id="scan";
           scan_point2.point.x = ranges[i] * cos(i * PI / 180) ;
           scan_point2.point.y =  ranges[i] * sin(i * PI / 180);
           scan_point2.point.z = z;

           cout<<"scanpoint: "<<scan_point2.point.x<<"   "<<scan_point2.point.y<<"   "<<endl;

           geometry_msgs::PointStamped odom_point;
           try{
                  listener.waitForTransform( "base_scan", "odom", ros::Time(0), ros::Duration(0.1) ); //必须加这个wait函数，因为数据传输有延迟，不加会报错
                  listener.transformPoint("odom",scan_point2,odom_point);

                  cout<<"odompoint: "<<odom_point.point.x<<"   "<<odom_point.point.y<<"   "<<endl;
              }
           catch (tf::TransformException &ex) 
           {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
           }

           scan_point.x = odom_point.point.x;
           scan_point.y = odom_point.point.y;
           scan_point.z = odom_point.point.z;
           
           points.points.push_back(scan_point);                   
      }
      
     
      lines.clear();                             
      extractLineR(ranges, lines); 

     marker_pub.publish(points);
     //marker_pub.publish(line_list);
               
}
