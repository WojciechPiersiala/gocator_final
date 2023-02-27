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

#define START ('1')
#define STOP ('2')
#define SEND  ('3')
#define INIT ('4')

#define PERS  ('4')
#define SAVE  ('5')

#define NORMAL ('1')
#define CARTESIAN ('2')

#define MODE CARTESIAN      
geometry_msgs::Pose get_pose(char pos, moveit::planning_interface::MoveGroupInterface *interface);

int main(int argc, char** argv){


    ros::init(argc, argv, "move_arm");
    ros::NodeHandle nh;
    ros::Publisher command_pub;
    std_msgs::String start_command, stop_command, send_command, pers_command,save_command;
    ros::AsyncSpinner spinner(1);
    spinner.start();    

    command_pub = nh.advertise<std_msgs::String>("scan_command",1);
    start_command.data = START;
    stop_command.data = STOP;
    send_command.data = SEND;

    pers_command.data = PERS;
    save_command.data = SAVE;

    static const std::string PLANNING_GROUP = "right_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO_NAMED("move_arm", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("move_arm", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("move_arm", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");            //use moveit_visual_tools::MoveItVisualTools in rviz
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    #if MODE == NORMAL

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success;
        while(ros::ok()){
            visual_tools.deleteAllMarkers();
            // // Go to start
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to start position");
            move_group_interface.setPoseTarget(get_pose(START));

            success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("move_arm", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
            visual_tools.publishAxisLabeled(get_pose(START), "start_pose");
            visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("right_arm_ee_link"), joint_model_group, rvt::LIME_GREEN);
            visual_tools.trigger();
            move_group_interface.execute(my_plan);
        

            // // Go to stop
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to stop position");
            move_group_interface.setPoseTarget(get_pose(STOP));

            success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("move_arm", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
            visual_tools.publishAxisLabeled(get_pose(STOP), "stop_pose");
            visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("right_arm_ee_link"), joint_model_group, rvt::LIME_GREEN);
            visual_tools.trigger();
            move_group_interface.execute(my_plan);
        } 
    #elif MODE == CARTESIAN

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success;
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    #if 0
        float j3_1, j4_1, j5_1;
        j3_1 = -0.959;
        j4_1 = -0.837;
        j5_1 = 1.91;

    #if 0 //if 1 use simaulaiton
        j4_1 = 0.01;
    #endif 


        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to revolve about the joints");
        joint_group_positions[3] = j3_1;
        joint_group_positions[4] = j4_1;
        joint_group_positions[5] = j5_1;
        move_group_interface.setJointValueTarget(joint_group_positions);
        success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "SUCCESS" : "FAILED");
        // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to change joint state");
        move_group_interface.execute(my_plan);

    #endif


        std::vector<geometry_msgs::Pose> waypoints;
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction;

        /////////////////////////////////
        // // Move to init position // //
        /////////////////////////////////

        visual_tools.prompt("Press 'next' to plan INIT");
        move_group_interface.setPoseTarget(get_pose(INIT, &move_group_interface));
        success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("move_arm", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        visual_tools.prompt("Press 'next' to go to INIT");
        move_group_interface.execute(my_plan);
        

        ///////////////////////////////////////
        // // Loop between work positions // //
        ///////////////////////////////////////

        while(ros::ok()){
            visual_tools.prompt("Press NEXT to plan to START ");
            waypoints.push_back(get_pose(START, &move_group_interface));
            waypoints.push_back(get_pose(STOP, &move_group_interface));
            fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
            visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
            visual_tools.trigger();
            waypoints.clear();
            visual_tools.prompt("Press NEXT to execute to START ");
            command_pub.publish(start_command);
            move_group_interface.execute(trajectory);
            command_pub.publish(send_command);


            visual_tools.prompt("Press NEXT to plan to STOP ");
            waypoints.push_back(get_pose(STOP, &move_group_interface));
            waypoints.push_back(get_pose(START, &move_group_interface));
            fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
            visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
            visual_tools.trigger();
            waypoints.clear();
            visual_tools.prompt("Press NEXT to execute to STOP ");
            command_pub.publish(start_command);
            move_group_interface.execute(trajectory);
            command_pub.publish(send_command);
        }

    #endif
    return 0;
}


geometry_msgs::Pose get_pose(char pos, moveit::planning_interface::MoveGroupInterface *interface){
    geometry_msgs::Pose target_pose1;
    tf2::Quaternion q;

    switch(pos){
        case(START):{
            // target_pose1.position.x =  0.60;  
            target_pose1.position.y = -0.55;  //-0.55 
            // target_pose1.position.z =  1.1; 
            target_pose1.orientation = interface->getCurrentPose().pose.orientation;
            target_pose1.position.x = interface->getCurrentPose().pose.position.x;
            target_pose1.position.z = interface->getCurrentPose().pose.position.z;
            break;
        }
        case(STOP):{
            // target_pose1.position.x =  0.60;  
            target_pose1.position.y =  0.10;  
            // target_pose1.position.z =  1.1; 
            target_pose1.orientation = interface->getCurrentPose().pose.orientation;
            target_pose1.position.x = interface->getCurrentPose().pose.position.x;
            target_pose1.position.z = interface->getCurrentPose().pose.position.z;
            break;
        }
        case(INIT):{
            target_pose1.position.x =  0.60;  
            target_pose1.position.y = -0.55;  
            target_pose1.position.z =  1.1; 

            q.setRPY(-M_PI/2, M_PI/2, M_PI);
            target_pose1.orientation.w = q.getW();
            target_pose1.orientation.x = q.getX();
            target_pose1.orientation.y = q.getY();
            target_pose1.orientation.z = q.getZ();
            break;
        }
        default:
            break;
    }
    return target_pose1;
}
