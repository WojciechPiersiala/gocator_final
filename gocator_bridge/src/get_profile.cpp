          /*
 * ReceiveProfile.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2020 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive range data in Profile Mode and translate to engineering units (mm). Gocator must be in Profile Mmode.
 * Ethernet output for the profile data must be enabled.
 *
 */

#include <ros/ros.h>
#include <string>
#include <vector>

#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <vector>
#include <custom_lib/custom.h>
#include <std_msgs/String.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>



#define RECEIVE_TIMEOUT         (20000000) 
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.    
#define SENSOR_IP               "192.168.1.10"                      

#define SCALE_COLOR(VALUE) ((int)(((VALUE)+100)*1.02))

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

#define START ('1')
#define STOP ('2')
#define MAX_CLD_SIZE (250000)


#define START ('1')
#define STOP  ('2')
#define SEND  ('3')
#define PERS  ('4')
#define SAVE  ('5')


void command_callback(const std_msgs::String &str);

bool recived = false;
char message = 0;
int main(int argc, char **argv){   
    std::string frame_name = "gocator_frame";
    std::string nodename = "get_surface";
    kAssembly api = kNULL;
    kStatus status;
    unsigned int i, j, k, arrayIndex;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoDataSet dataset = kNULL;
    GoStamp *stamp =kNULL;
    GoDataMsg dataObj;
    kIpAddress ipAddress;
    GoSetup setup = kNULL;
    k32u profilePointCount;

    ros::init(argc,argv,nodename);
    ros::NodeHandle nh;
    double t_now, t_duration;
    
    ros::Publisher profile_pub = nh.advertise<sensor_msgs::PointCloud2>("profile_points", 1);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cld(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cld(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 profile_cld_msg;
    sensor_msgs::PointCloud2 surface_cld_msg;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    float theta = 3.24954; 
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0, 0, 0.327; //3.26 //193
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));

    ros::Subscriber command_sub = nh.subscribe("scan_command",10, command_callback);

    if(argc > 1){
        t_duration = atof(argv[1]);
    }
    else{
        t_duration = 0.01f;
    }
    std::cout << t_duration;
    t_now = ros::Time::now().toSec();

    // construct Gocator API Library
    if ((status = GoSdk_Construct(&api)) != kOK){
        ROS_INFO("Error: GoSdk_Construct:%d\n", status);
        return 1;
    }

    // construct GoSystem object
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK){
        ROS_INFO("Error: GoSystem_Construct:%d\n", status);
        return 1;
    }

    // Parse IP address into address data structure
    kIpAddress_Parse(&ipAddress, SENSOR_IP);

    // obtain GoSensor object by sensor IP address
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK){
        ROS_INFO("Error: GoSystem_FindSensor:%d\n", status);
        return 1;
    }

    // create connection to GoSensor object
    if ((status = GoSensor_Connect(sensor)) != kOK){
        ROS_INFO("Error: GoSensor_Connect:%d\n", status);
        return 1;
    }

    // enable sensor data channel
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK){
        ROS_INFO("Error: GoSensor_EnableData:%d\n", status);
        return 1;
    }
    
    // retrieve setup handle
    if ((setup = GoSensor_Setup(sensor)) == kNULL){
        ROS_INFO("Error: GoSensor_Setup: Invalid Handle\n");
    }   

    // retrieve total number of profile points prior to starting the sensor
    if (GoSetup_UniformSpacingEnabled(setup)){
        // Uniform spacing is enabled. The number is based on the X Spacing setting
        profilePointCount = GoSetup_XSpacingCount(setup, GO_ROLE_MAIN);
        ROS_INFO("Status: Uniform spacing enabled\n");
    }
    else{
        // non-uniform spacing is enabled. The max number is based on the number of columns used in the camera. 
        profilePointCount = GoSetup_FrontCameraWidth(setup, GO_ROLE_MAIN);
        ROS_INFO("Status: Uniform spacing disabled\n");
    }


    while(ros::ok()){
        
        if(::recived){
            ::recived = false;
            
            if(::message == START){
                // ROS_INFO_STREAM("STARTED, message :" << ::message);
                // start Gocator sensor
                if ((status = GoSystem_Start(system)) != kOK){
                    ROS_INFO("Error: GoSensor_Start:%d\n", status);
                    return 1;
                }
            }
            else if(::message != START){
                // ROS_INFO_STREAM("STOPPED, message :" << ::message);
                if ((status = GoSystem_Stop(system)) != kOK){
                    ROS_INFO("Error: GoSensor_Start:%d\n", status);
                    return 1;
                }
            }
        }
        if(::message == START){
            // ROS_INFO_STREAM("WORKING, message :" << ::message);
            if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK){           
                // each result can have multiple data items
                // loop through all items in result message
                for (i = 0; i < GoDataSet_Count(dataset); ++i){           
                    dataObj = GoDataSet_At(dataset, i);
                    //Retrieve GoStamp message
                    switch(GoDataMsg_Type(dataObj)){
                    case GO_DATA_MESSAGE_TYPE_STAMP:{
                            GoStampMsg stampMsg = dataObj;
                        }
                        break;
                    case GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE:{                   
                            GoResampledProfileMsg profileMsg = dataObj;
                            for (k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k){
                                unsigned int validPointCount = 0;
                                short* data = GoResampledProfileMsg_At(profileMsg, k); 
                                double XResolution = NM_TO_MM(GoResampledProfileMsg_XResolution(profileMsg));
                                double ZResolution = NM_TO_MM(GoResampledProfileMsg_ZResolution(profileMsg));
                                double XOffset = UM_TO_MM(GoResampledProfileMsg_XOffset(profileMsg));
                                double ZOffset = UM_TO_MM(GoResampledProfileMsg_ZOffset(profileMsg));                                                    

                                pcl::PointXYZRGB point;
                                uint8_t R, G, B;
                                uint32_t RGB;
                                double X, Y, Z;

                                //translate 16-bit range data to engineering units and copy profiles to memory array
                                for (arrayIndex = 0; arrayIndex < GoResampledProfileMsg_Width(profileMsg); ++arrayIndex){
                                    if (data[arrayIndex] != INVALID_RANGE_16BIT ){

                                        validPointCount++;
                                        X = XOffset + XResolution * arrayIndex;
                                        Z = ZOffset + ZResolution * data[arrayIndex];

                                        point.x = X/1000.f;
                                        point.y = 0.f;
                                        point.z = Z/1000.f;

                                        point.rgb = custom::HSVtoRGB((255-SCALE_COLOR(Z)),100,100);
                                        profile_cld->points.push_back(point);
                                    }
                                    else{
                                    }
                                }
                            }
                        }
                        break;          
                    }
                }

                GoDestroy(dataset);
                profile_cld->width = (int)profile_cld->size();
                profile_cld->height = 1;
                profile_cld->is_dense = true;

                profile_cld->resize(profile_cld->height * profile_cld->width);

                pcl::transformPointCloud (*profile_cld, *transformed_cloud, transform_2);
                profile_cld = transformed_cloud;
            
                pcl::toROSMsg(*profile_cld, profile_cld_msg);
                profile_cld_msg.header.stamp = ros::Time::now();
                profile_cld_msg.header.frame_id = frame_name;   //"right_arm_gripper_link"
                profile_cld_msg.is_dense = true;    ///?
                profile_pub.publish(profile_cld_msg);
                profile_cld->clear();
            }
        }
        ros::spinOnce();
    }//loop


    // stop Gocator sensor
    if ((status = GoSystem_Stop(system)) != kOK){
        ROS_INFO("Error: GoSensor_Stop:%d\n", status);
        return 1;
    }

    // destroy handles
    GoDestroy(system);
    GoDestroy(api);
    return 0;
}



void command_callback(const std_msgs::String &str){
     assert(str.data.size() == 1);
    ::message = str.data[0];
    ::recived = true;
}