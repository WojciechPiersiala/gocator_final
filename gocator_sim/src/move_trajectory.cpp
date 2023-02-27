#include <ros/ros.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>


#define START ('1')
#define STOP  ('2')
#define SEND  ('3')
#define PERS  ('4')
#define SAVE  ('5')

#define SIM (0)
#define ASK (1)


void log(const std::string &text);

class MainProgram{
private:
    const std::string PLANNING_GROUP_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    const moveit::core::JointModelGroup* joint_model_group_;
    geometry_msgs::Pose init_pose_;

public:
    ros::Publisher command_pub_;
    geometry_msgs::Pose work_pose_;
public:
    MainProgram():
    PLANNING_GROUP_("right_arm"),
    move_group_interface_(PLANNING_GROUP_),
    joint_model_group_(move_group_interface_.getCurrentState()->getJointModelGroup(PLANNING_GROUP_))
        {
        ROS_INFO_NAMED("move_arm", "Planning frame: %s", move_group_interface_.getPlanningFrame().c_str());
        ROS_INFO_NAMED("move_arm", "End effector link: %s", move_group_interface_.getEndEffectorLink().c_str());
        ROS_INFO_NAMED("move_arm", "Available Planning Groups:");
        std::copy(move_group_interface_.getJointModelGroupNames().begin(),move_group_interface_.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

        // // set init pose
        #if SIM
            init_pose_.position.x =  0.4133543321528152;
            init_pose_.position.y = -0.2659727718740041;
            init_pose_.position.z =  1.0805978702774814;
        #else
            init_pose_.position.x =  0.600165 ;
            init_pose_.position.y = -0.117877;
            init_pose_.position.z =  1.15583;
        #endif
        tf2::Quaternion quat;
        quat.setW(0.00958114871271333);
        quat.setX(-0.9345645553077441);
        quat.setY(-0.3188656138332367);
        quat.setZ(-0.1575500360727876);
        init_pose_.orientation.w = quat.getW();
        init_pose_.orientation.x = quat.getX();
        init_pose_.orientation.y = quat.getY();
        init_pose_.orientation.z = quat.getZ();


        // // set work pose 
        work_pose_.position.x =  0.5;
        work_pose_.position.y = -0.4;
        work_pose_.position.z =  1.1;
        #if SIM
            quat.setW(0.00958114871271333);
            quat.setX(-0.9345645553077441);
            quat.setY(-0.3188656138332367);
            quat.setZ(-0.1575500360727876);
        #else
            quat.setRPY(-M_PI/2, M_PI/2, M_PI);
        #endif
        work_pose_.orientation.w = quat.getW();
        work_pose_.orientation.x = quat.getX();
        work_pose_.orientation.y = quat.getY();
        work_pose_.orientation.z = quat.getZ();
    }


    void display_current_pose(){
        geometry_msgs::Pose curr_pose;
        curr_pose = move_group_interface_.getCurrentPose().pose;
        auto T = curr_pose.position;
        auto R = curr_pose.orientation;

        ROS_INFO_STREAM("Current pose translation: [" << T.x <<" "<< T.y << " " << T.z << "orientation : "<<R.w <<" "<<R.x <<" "<< R.y<< " "<<R.z);
    }


   void change_orinentation(double roll_in=0, double pitch_in=0, double yaw_in=0){
        geometry_msgs::Pose curr_pose;
        tf2::Quaternion quat;
        geometry_msgs::Quaternion quat_msg;
        double roll, pitch, yaw;
        const double rad2deg = 0.0174532925;

        curr_pose = move_group_interface_.getCurrentPose().pose;

        quat.setX(curr_pose.orientation.x);
        quat.setY(curr_pose.orientation.y);
        quat.setZ(curr_pose.orientation.z);
        quat.setW(curr_pose.orientation.w);

        tf2::Matrix3x3 mat(quat);
        mat.getRPY(roll, pitch, yaw);

        std::cout <<"\033[1;33;40m" <<" angle " <<roll << " " << pitch << " " << yaw << " "  << "\033[0m" << std::endl;
        roll  += roll_in  * rad2deg;
        pitch += pitch_in * rad2deg;
        yaw   += yaw_in   * rad2deg;

        std::cout <<"\033[1;33;40m" <<" angle " <<roll << " " << pitch << " " << yaw << " "  << "\033[0m" << std::endl;
        
        quat.setRPY(roll, pitch, yaw);

        curr_pose.orientation.x = quat.getX();
        curr_pose.orientation.y = quat.getY();
        curr_pose.orientation.z = quat.getZ();
        curr_pose.orientation.w = quat.getW();

        ROS_INFO_STREAM("Changing orientation...");

        set_pose(curr_pose);
    }



    void set_pose(const geometry_msgs::Pose &goal, bool ask=ASK){
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_interface_.setPoseTarget(goal);
        bool success = (move_group_interface_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("planing :" << success ? "SUCCESS" : "FAILED");
        if(success){
            if(ask){
                char resp;
                std::cout << "\033[1;33;40m"  << "Execute trajectory [Y/n]?" << "\033[0m"; std::cin>>resp;
                if( resp == 'Y' || resp == 'y'){
                    move_group_interface_.execute(plan);
                    ROS_INFO_STREAM("Moved to the next position");
                }
                else{
                    std::cout <<"\033[1;31;40m" << "STOP" <<"\033[0m";
                    return; // do nothing
                }
            }
            else{
                move_group_interface_.execute(plan);
                ROS_INFO_STREAM("Moved to the next position");
            }
        }
    }


    void go_to_init(){
        ROS_INFO_STREAM("Moving to initial pose");
        set_pose(init_pose_);
    }


    void change_position(double dx=0, double dy=0, double dz=0){
        geometry_msgs::Pose curr_pose;
        curr_pose = move_group_interface_.getCurrentPose().pose;
        curr_pose.position.x += dx;
        curr_pose.position.y += dy;
        curr_pose.position.z += dz;

        set_pose(curr_pose);
    }

    void follow_cartesian_trajectory(bool ask=ASK, bool scan=false){
        geometry_msgs::Pose curr_pose;
        std::vector<geometry_msgs::Pose> waypoints;
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction;
        bool success;
        const double dist = 0.15;

        curr_pose = move_group_interface_.getCurrentPose().pose;
        curr_pose.position.y += dist;
        waypoints.push_back(curr_pose);
        curr_pose.position.y -= dist;
        waypoints.push_back(curr_pose);

        fraction = move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_STREAM("cartesian planning : "<< fraction * 100.f << "achieved");
        waypoints.clear();
        fraction > 0.95 ? success=true : success=false;

        if(success){
            ROS_INFO_STREAM("SUCCESS");
            if(ask){
                char resp;
                std::cout << "\033[1;33;40m"  << "Execute trajectory [Y/n]?" << "\033[0m"; std::cin>>resp;
                if( resp == 'Y' || resp == 'y'){
                    if(scan){
                        std_msgs::String start_command;
                        start_command.data = START;
                        command_pub_.publish(start_command);
                    }
                    move_group_interface_.execute(trajectory);
                }
                else{
                    std::cout <<"\033[1;31;40m" << "STOP" <<"\033[0m";
                    return; // do nothing
                }
            }
            else{
                if(scan){
                    std_msgs::String start_command;
                    start_command.data = START;
                    command_pub_.publish(start_command);
                }
                move_group_interface_.execute(trajectory);
                ROS_INFO_STREAM("Scan completed");
            }
        }
    }


    void scan_surface(){
        std_msgs::String start_command, send_command, persist_command, save_command;
        start_command.data = START;
        send_command.data = SEND;
        persist_command.data = PERS;
        save_command.data = SAVE;

        follow_cartesian_trajectory(ASK,true);

        ros::Duration(0.1).sleep();
        command_pub_.publish(send_command);
        ros::Duration(2).sleep();
        command_pub_.publish(persist_command);
        ros::Duration(2.f).sleep();
        command_pub_.publish(save_command);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "move_trajectory");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start(); 

    MainProgram Program;
    Program.command_pub_ = nh.advertise<std_msgs::String>("scan_command",1);


    
    Program.scan_surface();
    log("display current pose");
    Program.display_current_pose();
    
    log("go to initial position [0]");
    Program.go_to_init();

    log("set work pose [1]");
    Program.set_pose(Program.work_pose_, ASK);



    Program.scan_surface();

    log("change orientation [-Y]");
    Program.change_orinentation(0,-5,0);


    log("change orientation [-Y]");
    Program.change_orinentation(0,-5,0);
    Program.scan_surface();
    log("change orientation [-Y]");
    Program.change_orinentation(0,-5,0);
    Program.scan_surface();
    log("change orientation [-Y]");
    Program.change_orinentation(0,-5,0);
    Program.scan_surface();
    log("change orientation [Y]");
    Program.change_orinentation(0,20,0);


    log("go closer to surface [5]");
    Program.change_position(0,0,-0.02);
    Program.scan_surface();


    log("change orientation [-Y]");
    Program.change_orinentation(0,5,0);
    Program.scan_surface();
    log("change orientation [-Y]");
    Program.change_orinentation(0,5,0);
    Program.scan_surface();
    log("change orientation [-Y]");
    Program.change_orinentation(0,5,0);
    Program.scan_surface();
    log("change orientation [-Y]");
    Program.change_orinentation(0,5,0);
    Program.scan_surface();
    log("change orientation [Y]");
    Program.change_orinentation(0,-20,0);

    log("go closer to surface [5]");
    Program.change_position(0,0,-0.02);
    Program.scan_surface();


    log("change orientation [-Y]");
    Program.change_orinentation(0,-5,0);
    Program.scan_surface();
    log("change orientation [-Y]");
    Program.change_orinentation(0,-5,0);
    Program.scan_surface();
    log("change orientation [-Y]");
    Program.change_orinentation(0,-5,0);
    Program.scan_surface();
    log("change orientation [-Y]");
    Program.change_orinentation(0,-5,0);
    Program.scan_surface();
    log("change orientation [Y]");
    Program.change_orinentation(0,20,0);



    Program.set_pose(Program.work_pose_, ASK);
    return 0;
}

void log(const std::string &text){
    std::cout <<"\n \033[1;32;40m" << text << "\033[0m" << std::endl;
}
