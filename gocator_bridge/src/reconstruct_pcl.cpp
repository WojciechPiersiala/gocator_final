# include <iostream>
# include <string>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>

#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

static const std::string pcd_path = "/robot40human_ws/src/gocator/gocator_bridge/clouds/";
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

namespace fs = boost::filesystem;

class NamedCloud{
public:

    NamedCloud(){}

    void log(){
        std::cout << " trans_type " <<  trans_type << " \n";
        std::cout << " head_id " <<  head_id << " \n";
 
  std::cout << " head_seq : " <<  head_seq << " \n";
  std::cout << " head_stamp : " <<  head_stamp<< " \n";
  std::cout << " tran_X : " <<  tran_X<< " \n";
  std::cout << " tran_Y : " <<  tran_Y<< " \n";
  std::cout << " tran_Z : " <<  tran_Z<< " \n";
  std::cout << " rot_W : " <<  rot_W<< " \n";
  std::cout << " rot_X : " <<  rot_X<< " \n";
  std::cout << " rot_Y : " <<  rot_Y<< " \n";
  std::cout << " rot_Z : " <<  rot_Z<< " \n";

    }
public:
    std::string trans_type; 
    std::string head_id;
    float head_seq;
    float head_stamp;
    float tran_X;
    float tran_Y;
    float tran_Z;
    float rot_W;
    float rot_X;
    float rot_Y;
    float rot_Z;
};


class CalibSurface{
public:
    CalibSurface(){
        cloud=      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        msg=      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

    void init(){

        pcl::io::loadPCDFile<pcl::PointXYZ> (cld_file_name, *cloud);
        msg->header.frame_id = "map";
        msg->height = cloud->height;
        msg->width = cloud->width;

        for (size_t i = 0; i < cloud->size (); i++) {
            x_cloud = cloud->points[i].x;
            y_cloud = cloud->points[i].y;
            z_cloud = cloud->points[i].z;
            msg->points.push_back (pcl::PointXYZ (x_cloud, y_cloud, z_cloud));
        }
    }


    void run(){
        cld_pub.publish(msg);
    }

    void sendTransform(){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(1.0, 2.0, 3.0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "turtle_name"));
    }

    NamedCloud NC1;
    NamedCloud NC2; 
    std::string cld_name;
    std::string cld_file_name;

    ros::Publisher cld_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    PointCloud::Ptr msg;
    double x_cloud; double y_cloud; double z_cloud;

private:
    ros::NodeHandle nh_;
    std::string parent_frame_id_;
    std::string child_frame_id_;
    boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
};



void readData(std::ifstream &inFile, const std::string file_name, NamedCloud &NC1, NamedCloud &NC2);
std::vector<std::string>  processFile(std::ifstream &inFile);



int main(int argc, char** argv){

ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    
    NamedCloud NC1, NC2;
    int count = 0;

    std::vector<CalibSurface> calibSurfaces;
    std::vector<std::string> cloud_files;

    for (const auto & entry : fs::directory_iterator(pcd_path)){
        
        const std::string file_path = entry.path().string();
        if(file_path.find(".txt") != std::string::npos){
            std::ifstream inFile;
            readData(inFile, file_path,NC1,NC2);
            CalibSurface surface;
            surface.NC1 = NC1;
            surface.NC2 = NC2;
           

            std::string ext =".txt";
            std::string cld_path = file_path.substr(0, file_path.length() - ext.length());
            surface.cld_file_name = cld_path+".pcd";
            calibSurfaces.push_back(surface);
        }
        
    }
     std::cout << "before";
    for(size_t it=0; it < calibSurfaces.size(); it++){
        std::cout <<calibSurfaces[it].cld_file_name << std::endl;
        calibSurfaces[it].NC1.log();
        calibSurfaces[it].NC2.log();

        std::cout << "=================================\n";
    }
    std::cout << "after";

    CalibSurface test =calibSurfaces[0];

    for(int i=0; i<calibSurfaces.size(); i++){
        calibSurfaces[i].cld_pub = nh.advertise<PointCloud> ("cloud_" +std::to_string(i) , 1);    
        calibSurfaces[i].init();
    }
    
    ros::Rate loop_rate(1);
    while(nh.ok()){
        for(int i=0; i<calibSurfaces.size(); i++){
            calibSurfaces[i].run();
        }
        loop_rate.sleep ();
    }
    return 0;
}





void readData(std::ifstream &inFile, const std::string file_name, NamedCloud &NC1, NamedCloud &NC2){
    inFile.open(file_name);
    if(inFile.is_open()){
        std::vector<std::string> data_vec;
        data_vec = processFile(inFile);
        NC1.trans_type=            data_vec[0];
        NC1.head_id=               data_vec[1];
        NC1.head_seq=    std::stof(data_vec[2]);
        NC1.head_stamp=  std::stof(data_vec[3]);
        NC1.tran_X=      std::stof(data_vec[4]);
        NC1.tran_Y=      std::stof(data_vec[5]);
        NC1.tran_Z=      std::stof(data_vec[6]);
        NC1.rot_W=       std::stof(data_vec[7]);
        NC1.rot_X=       std::stof(data_vec[8]);
        NC1.rot_Y=       std::stof(data_vec[9]);
        NC1.rot_Z=       std::stof(data_vec[10]);

        NC2.trans_type=            data_vec[11];
        NC2.head_id=               data_vec[12];
        NC2.head_seq=    std::stof(data_vec[13]);
        NC2.head_stamp=  std::stof(data_vec[14]);
        NC2.tran_X=      std::stof(data_vec[15]);
        NC2.tran_Y=      std::stof(data_vec[16]);
        NC2.tran_Z=      std::stof(data_vec[17]);
        NC2.rot_W=       std::stof(data_vec[18]);
        NC2.rot_X=       std::stof(data_vec[19]);
        NC2.rot_Y=       std::stof(data_vec[20]);
        NC2.rot_Z=       std::stof(data_vec[21]);
    }
    else{
        std::cout <<" failed to open\n";
        exit(-1);
    }
}



std::vector<std::string>  processFile(std::ifstream &inFile){
    std::vector<std::string> data_vec;
    std::string line, word;
    std::istringstream iss;
    while(!inFile.eof()){
        std::getline(inFile, line);
        if(inFile.good()){
            iss.clear();
            iss.str(line);
            while(iss.good()){
                iss >> word;
                data_vec.push_back(word);
            }
        }
    }
    return data_vec;
}