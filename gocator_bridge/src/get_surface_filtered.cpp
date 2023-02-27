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
#include <pcl/filters/voxel_grid.h>

#include <iostream>
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
class TransformListener{
public:
    TransformListener() {}
    geometry_msgs::TransformStamped get_transform(const std::string frame1, const std::string frame2){
        static tf2_ros::Buffer buffer;
        static tf2_ros::TransformListener listener(buffer);
        geometry_msgs::TransformStamped transformStamped;

        try{
            transformStamped = buffer.lookupTransform(frame1, frame2,ros::Time(0)); 
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
static int num = 0;

class SurfaceListener{
public:
    double t_now2_, t_duration2_;
    double angle_;
    char pressed_key_;
    char action_;

    std::string command_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cld_;
    sensor_msgs::PointCloud2 surface_cld_msg_;
    pcl::PCLPointCloud2 surface_cld_cld2_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr persistent_cld_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr persistent_cld_fil_;
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

        profile_cld_        =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        surface_cld_        =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        persistent_cld_     =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        profile_cld_2_      =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        persistent_cld_fil_ =      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    }


    void command_callback(const std_msgs::String &str){
        command_ = str.data;
        assert(command_.size() == 1);
        action_ = command_[0];
        if (action_ == START){
            surface_cld_->clear();
        }
        ROS_INFO_STREAM("[get_surface_robot], Recived input : " << action_ << "\n");
    }



    void profile_callback(const sensor_msgs::PointCloud2 &cld){
        pcl_conversions::toPCL(cld, profile_cld_cld2_);
        pcl::fromPCLPointCloud2(profile_cld_cld2_, *profile_cld_);
    }


    void run(){
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
                pcl::toROSMsg(*surface_cld_, surface_cld_msg_);
                surface_cld_msg_.header.stamp = ros::Time::now();
                surface_cld_msg_.header.frame_id = "map";   
                surface_cld_msg_.is_dense = true;    
                surface_pub_.publish(surface_cld_msg_);
                *persistent_cld_ += *surface_cld_;
                ROS_INFO_STREAM("sent, STOP");
                action_ = STOP;
                break;
            }
            case PERS:{
                ROS_INFO_STREAM("sending scanned persistent");

                ROS_INFO_STREAM("Size before downsampling: " << persistent_cld_->size());
                downsample(persistent_cld_, persistent_cld_fil_, 0.01);
                ROS_INFO_STREAM("Size after downsampling: " << persistent_cld_fil_->size());


                pcl::toROSMsg(*persistent_cld_fil_, persistent_cld_msg_);
                persistent_cld_msg_.header.stamp = ros::Time::now();
                persistent_cld_msg_.header.frame_id = "map";   
                persistent_cld_msg_.is_dense = true;    
                persistent_pub_.publish(persistent_cld_msg_);
                action_ = STOP;
                break;
            }
            case SAVE:{
                ROS_INFO_STREAM("Saving scanned persistent");

                geometry_msgs::TransformStamped gripper_trans, gocator_trans;
                gripper_trans = transform_listener_.get_transform("base_link", "right_arm_gripper_link"); 
                gocator_trans = transform_listener_.get_transform("right_arm_gripper_link", "gocator_frame"); 

                std::ofstream text_file(::pcd_path+::pcd_name + std::to_string(num) + ".txt");
                std::string file = ::pcd_path + ::pcd_name + std::to_string(num) + ".pcd";

                pcl::io::savePCDFileASCII (file, *persistent_cld_fil_);
                text_file <<  log_transform(gripper_trans, "gripper_trans") << log_transform(gocator_trans, "gocator_trans");
                text_file.close();

                std::cerr << "Saved " << persistent_cld_fil_->size() << " to: " << file << std::endl;
                persistent_cld_->clear();
                persistent_cld_fil_->clear();
                action_ = STOP;
                ::num ++;
                break;
            }
            case STOP:
                break;
            default:
                break;
        }
    }

    void downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cld_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cld_out, float leaf=0.01f){
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cld_in);
        sor.setLeafSize(leaf, leaf, leaf);
        sor.filter(*cld_out);
    }

    std::string log_transform(const geometry_msgs::TransformStamped &trans, const std::string &trans_name){
        auto H = trans.header;
        auto T = trans.transform.translation;
        auto R = trans.transform.rotation;
        std::string head_str = H.frame_id + " " + std::to_string(H.seq) + " " + std::to_string(H.stamp.toSec());
        std::string transl_str = std::to_string(T.x) + " " + std::to_string(T.y) + " " + std::to_string(T.z);
        std::string rot_str = std::to_string(R.w) + " " + std::to_string(R.x) + " " + std::to_string(R.y) + " " + std::to_string(R.z);
        std::string out_str = trans_name +"\n" + head_str + "\n" +transl_str + "\n" + rot_str+ "\n";
        std::cout << out_str << std::endl;

        return out_str;
    }
};



int main(int argc, char ** argv){    
    ros::init(argc, argv, "get_surface_filtered");
    ros::NodeHandle nh;
    SurfaceListener surface_listener;

    surface_listener.persistent_pub_ = nh.advertise<sensor_msgs::PointCloud2>("persistent_points", 10);
    surface_listener.surface_pub_ = nh.advertise<sensor_msgs::PointCloud2>("surface_points", 10);
    ros::Subscriber profile_cld_sub = nh.subscribe("profile_points",10, &SurfaceListener::profile_callback, &surface_listener);
    ros::Subscriber command_sub = nh.subscribe("scan_command",10, &SurfaceListener::command_callback, &surface_listener);

    while(ros::ok()){
        surface_listener.run();
        ros::spinOnce();
    }
    return 0;
}