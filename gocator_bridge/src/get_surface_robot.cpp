#include <ros/ros.h>
#include <std_msgs/String.h>
#include <custom_lib/custom.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <custom_lib/custom.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

#include <tf2_ros/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <fstream>
#define MAX_STEP (250)
#define MAX_CLD_SIZE (250000000)

#define START ('1')
#define STOP  ('2')
#define SEND  ('3')
#define PERS  ('4')
#define SAVE  ('5')


#define AUTO ('1')
#define MANUAL ('0')

#define MODE AUTO

int file_num = 0;
bool file_first = true;

void log_transform(const geometry_msgs::TransformStamped &trans, const std::string &file_path, int count, const std::string &extra_path);

class TransformListener{
public:
    TransformListener() {}
    geometry_msgs::TransformStamped get_transform(const std::string frame1, const std::string frame2){
        static tf2_ros::Buffer buffer;
        static tf2_ros::TransformListener listener(buffer);
        geometry_msgs::TransformStamped transformStamped;

        try{
            transformStamped = buffer.lookupTransform(frame1, frame2,ros::Time(0)); //gocator_frame
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        return transformStamped;
    }
};



static const std::string pcd_path = "/robot40human_ws/src/gocator/gocator_bridge/clouds/";
static const std::string pcd_name = "obj";

class SurfaceListener{
public:
    double t_now2_, t_duration2_;
    double angle_;
    char pressed_key_;
    char action_;

    std::string command_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cld_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cld_tmp_;
    sensor_msgs::PointCloud2 surface_cld_msg_;
    pcl::PCLPointCloud2 surface_cld_cld2_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr persistent_cld_;
    sensor_msgs::PointCloud2 persistent_cld_msg_;
    pcl::PCLPointCloud2 persistent_cld_cld2_;


    sensor_msgs::PointCloud2 profile_cld_msg_;
    pcl::PCLPointCloud2 profile_cld_cld2_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cld_;
    ros::Publisher surface_pub_;
    ros::Publisher persistent_pub_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr profile_cld_2_;                                                                                                                                                                                        
    TransformListener transform_listener_;
public:

    SurfaceListener(){
        t_duration2_ = 0.0001;
        t_now2_ = ros::Time::now().toSec();
        action_ = 0;

        profile_cld_     =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        surface_cld_     =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        surface_cld_tmp_    =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        persistent_cld_  =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        profile_cld_2_   =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    }


    void command_callback(const std_msgs::String &str){
        command_ = str.data;
        assert(command_.size() == 1);
        action_ = command_[0];
        if (action_ == START){
            surface_cld_->clear();
            surface_cld_tmp_->clear();
        }
        ROS_INFO_STREAM("[get_surface_robot], Recived input : " << action_ << "\n");
    }



    void profile_callback(const sensor_msgs::PointCloud2 &cld){
        pcl_conversions::toPCL(cld, profile_cld_cld2_);
        pcl::fromPCLPointCloud2(profile_cld_cld2_, *profile_cld_);

        #if MODE == MANUAL

                if(pressed_key_ = custom::getch()){
                    action_ = pressed_key_;
                    if(action_ == START)
                        surface_cld_->clear();
                        ROS_INFO_STREAM("recived input : " << action_ << "\n");
                }
        #endif 
        switch(action_){
            case START:{
                if((ros::Time::now().toSec() - t_now2_)> t_duration2_ && surface_cld_->size() < MAX_CLD_SIZE){
                    t_now2_ = ros::Time::now().toSec();
                    geometry_msgs::TransformStamped transform;
                    transform = transform_listener_.get_transform("map", "gocator_frame"); 
                    geometry_msgs::Transform transform2;
                    transform2.rotation= transform.transform.rotation;
                    transform2.translation= transform.transform.translation;
                    pcl_ros::transformPointCloud(*profile_cld_, *profile_cld_2_, transform2);
                    *surface_cld_ += *profile_cld_2_;
                }
                break;
            }
            case SEND:{
                ROS_INFO_STREAM("sending scanned surface");
                int step =1000;
                pcl::toROSMsg(*surface_cld_, surface_cld_msg_);
                surface_cld_msg_.header.stamp = ros::Time::now();
                surface_cld_msg_.header.frame_id = "map";   
                surface_cld_msg_.is_dense = true;    
                surface_pub_.publish(surface_cld_msg_);


                const std::string file_path = "/robot40human_ws/src/gocator/gocator_bridge/surface_cld/";
                std::string file = file_path + "cld" + std::to_string(file_num)+".pcd";
                ROS_INFO_STREAM("saving " << surface_cld_->size() <<"points to: " << file);
                auto res = pcl::io::savePCDFileASCII (file, *surface_cld_);
                ROS_INFO_STREAM("saved, res: " << res);

                log_transform(transform_listener_.get_transform("map", "gocator_frame"),file_path + "transforms.txt",file_num,file);
                ROS_INFO_STREAM("saved, transform: ");
                file_num++;
                action_ = STOP;
                break;
            }
            case PERS:{
                ROS_INFO_STREAM("sending scanned persistent");
                pcl::toROSMsg(*persistent_cld_, persistent_cld_msg_);
                persistent_cld_msg_.header.stamp = ros::Time::now();
                persistent_cld_msg_.header.frame_id = "map";   
                persistent_cld_msg_.is_dense = true;    
                persistent_pub_.publish(persistent_cld_msg_);
                action_ = STOP;
                break;
            }
            case SAVE:{
                ROS_INFO_STREAM("Saving scanned persistent");
                static int num = 0;
                static std::string file = ::pcd_path + ::pcd_name + std::to_string(num) + ".pcd";
                num ++;
                pcl::io::savePCDFileASCII (file, *persistent_cld_);
                std::cerr << "Saved " << persistent_cld_->size() << " to: " << file << std::endl;
                persistent_cld_->clear();
                action_ = STOP;
                break;
            }
            case STOP:
                break;
            default:
                break;
        }
    }
};

void log_transform(const geometry_msgs::TransformStamped &trans, const std::string &file_path, int count, const std::string &extra_path){
    std::string tr_x = std::to_string(trans.transform.translation.x);
    std::string tr_y = std::to_string(trans.transform.translation.y);
    std::string tr_z = std::to_string(trans.transform.translation.z);
    
    std::string rot_x = std::to_string(trans.transform.rotation.x);
    std::string rot_y = std::to_string(trans.transform.rotation.y);
    std::string rot_z = std::to_string(trans.transform.rotation.z);
    std::string rot_w = std::to_string(trans.transform.rotation.w);

    std::string res = std::to_string(count) + ";" +tr_x +";" + tr_y +";" + tr_z +";" + rot_x+";" + rot_y+";" + rot_z + ";" + rot_w+";"+extra_path+"\n";
    if(file_first){
        std::ofstream file(file_path, std::ios::trunc);
        file << res;
        file_first = false;
    }
    else{
        std::ofstream file(file_path, std::ios::app);
        file << res;
    }
}


int main(int argc, char ** argv){    

    ros::init(argc, argv, "get_surface_robot");
    ros::NodeHandle nh;
    SurfaceListener surface_listener;

    surface_listener.persistent_pub_ = nh.advertise<sensor_msgs::PointCloud2>("persistent_points", 10);
    surface_listener.surface_pub_ = nh.advertise<sensor_msgs::PointCloud2>("surface_points", 10);
    ros::Subscriber profile_cld_sub = nh.subscribe("profile_points",10, &SurfaceListener::profile_callback, &surface_listener);
    ros::Subscriber command_sub = nh.subscribe("scan_command",10, &SurfaceListener::command_callback, &surface_listener);

    ros::spin();
    return 0;
}