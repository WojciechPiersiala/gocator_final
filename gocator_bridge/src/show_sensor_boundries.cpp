#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Dense>
#include "custom_lib/custom.h"

class Point{
    public: 
    Point(float x, float y);
    float getX(){return x_;}
    float getZ(){return z_;}
    friend Point getParams(Point p1, Point p2);

    float x_; 
    float y_;
    float z_;
};

Point getParams(Point p1, Point p2){
    float a = (p2.z_ - p1.z_)/(p2.x_ - p1.x_);
    float b =p1.z_ - a*p1.x_;
    Point p(a,b);
    return p;
}

Point::Point(float x, float y) : x_(x), z_(y) {
    y_ = 0.f;
}


float getAngle(const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2){
    float x1 = p1.x;  float x2 = p2.x;
    float y1 = p1.y;  float y2 = p2.y;
    float z1 = p1.z;  float z2 = p2.z;

    float dot = x1*x2 + y1*y2 + z1*z2 ;  
    float lenSq1 = x1*x1 + y1*y1 + z1*z1;
    float lenSq2 = x2*x2 + y2*y2 + z2*z2;
    float angle = acos(dot/sqrt(lenSq1 * lenSq2));
    return angle;
}


int main(int argc, char** argv )
{
    ros::init(argc, argv, "show_sensor_boundries");
    ros::NodeHandle n;
    ros::Rate r(10);

    std::string frame_name = "gocator_frame";
    Point p1(-0.98 , -0.85);
    Point p2( 1.16 , -0.64);
    Point p3( 0.38 ,  1.39);
    Point p4(-0.65 ,  1.29);
    p1.x_ /= 10; p2.x_ /= 10; p3.x_ /= 10; p4.x_ /= 10;
    p1.y_ /= 10; p2.y_ /= 10; p3.y_ /= 10; p4.y_ /= 10;
    p1.z_ /= 10; p2.z_ /= 10; p3.z_ /= 10; p4.z_ /= 10;
    std::cout <<" point1: " <<  p1.getX() << " " <<  p1.getZ() <<std::endl;
    std::cout <<" point2: " <<  p2.getX() << " " <<  p2.getZ() <<std::endl;
    std::cout <<" point3: " <<  p3.getX() << " " <<  p3.getZ() <<std::endl;
    std::cout <<" point4: " <<  p4.getX() << " " <<  p4.getZ() <<std::endl;
    Point parL1 = getParams(p1, p4);
    Point parL2 = getParams(p3, p2);
    float x5, y5;
    x5= (parL1.getZ() - parL2.getZ())/(parL2.getX() - parL1.getX());
    y5= parL1.getX()*x5 + parL1.getZ();
    Point p5(x5,y5);
    std::cout <<" l1 = " <<  parL1.getX() << "x + " <<  parL1.getZ() <<std::endl;
    std::cout <<" l2 = " <<  parL2.getX() << "x + " <<  parL2.getZ() <<std::endl;
    std::cout <<" point5: " <<  p5.getX() << " " <<  p5.getZ() <<std::endl;
    // middle point
    float x6, y6;
    x6 = (p3.getX() - p4.getX())/2 + p4.getX();
    y6 = (p3.getZ() - p4.getZ())/2 + p4.getZ();
    Point p6(x6,y6);
    std::cout <<" point6: " <<  p6.getX() << " " <<  p6.getZ() <<std::endl;
    //distance 
    float dist = sqrt(pow((x5-x6),2)+pow((y5-y6),2));
    std::cout <<"dist : " <<dist <<std::endl;

    ros::Publisher geometry_pub = n.advertise<geometry_msgs::PolygonStamped>("polygon",1);
    geometry_msgs::PolygonStamped my_polygon;
    float theta;
    Eigen::Matrix3f R;
    geometry_msgs::Point32 T;

    geometry_msgs::Point32 P1, P2, P3, P4, P5, P6;
    geometry_msgs::Point32 V1, V2;
    
    P1.x=p1.x_;  P2.x=p2.x_;  P3.x=p3.x_;  P4.x=p4.x_;  P5.x=p5.x_;  P6.x=p6.x_;
    P1.y=p1.y_;  P2.y=p2.y_;  P3.y=p3.y_;  P4.y=p4.y_;  P5.y=p5.y_;  P6.y=p6.y_;
    P1.z=p1.z_;  P2.z=p2.z_;  P3.z=p3.z_;  P4.z=p4.z_;  P5.z=p5.z_;  P6.z=p6.z_;

    V1.x = P6.x-P5.x;  V2.x =0.f; 
    V1.y = P6.y-P5.y;  V2.y =0.f; 
    V1.z = P6.z-P5.z;  V2.z =1.f; 

    std::cout << "Vectors: V1: [" << V1.x <<" " << V1.y<<" " << V1.z<< "] V2: [" << V2.x <<" " << V2.y<< " " << V2.z<< "]" <<std::endl;
    my_polygon.polygon.points.push_back(P1);
    my_polygon.polygon.points.push_back(P2);
    my_polygon.polygon.points.push_back(P3);
    my_polygon.polygon.points.push_back(P6);
    my_polygon.polygon.points.push_back(P5);
    my_polygon.polygon.points.push_back(P6);
    my_polygon.polygon.points.push_back(P4);

    theta = getAngle(V1, V2);
    theta += 0.215590939730827;   //offset 

    
    float ct = cos(theta);
    float st = sin(theta);
    
    R << ct, 0, st,
          0, 1, 0,
        -st, 0, ct;
    
    T.x =-P5.x;
    T.y =-P5.y;
    T.z =-P5.z;

    std::cout << "theta :" << theta << std::endl;
    std::cout << "R :" << R << std::endl;
    std::cout << "T: [" << V1.x <<" " << V1.y<<" " << V1.z<< "]" <<std::endl;

    std::vector<geometry_msgs::Point32, std::allocator<geometry_msgs::Point32>> *point_ref;
    point_ref =  &my_polygon.polygon.points;

    for(size_t it=0; it < (*point_ref).size(); it++){


        custom::transform((*point_ref)[it],R,T);
    }

    my_polygon.header.frame_id = frame_name;

    while (ros::ok()){
        geometry_pub.publish(my_polygon);
        r.sleep();
    }
    
    return 0;
}
