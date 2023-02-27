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

#define ROTATE_BY_JOINTS (1)

geometry_msgs::Pose get_pose(char pos);

static float Roll, Pitch, Yaw;

int main(int argc, char** argv){

    ros::init(argc, argv, "move_arm_init");
    ros::NodeHandle nh;
    ros::Publisher command_pub;
    std_msgs::String start_command, stop_command, send_command;
    ros::AsyncSpinner spinner(1);
    spinner.start();    

    static const std::string PLANNING_GROUP = "right_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO_NAMED("move_arm_init", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("move_arm_init", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("move_arm_init", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");            //use moveit_visual_tools::MoveItVisualTools in rviz
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;

#if ROTATE_BY_JOINTS 
    joint_group_positions[3] = -0.959;   // 1.27
    move_group_interface.setJointValueTarget(joint_group_positions);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "SUCCESS" : "FAILED");
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to change joint state");
    move_group_interface.execute(my_plan);

    joint_group_positions[4] =-0.837;  // 0.01   //-0.837
    move_group_interface.setJointValueTarget(joint_group_positions);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "SUCCESS" : "FAILED");
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to change joint state");
    move_group_interface.execute(my_plan);

    joint_group_positions[5] = 1.91;  // -1/6 turn in radians
    move_group_interface.setJointValueTarget(joint_group_positions);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "SUCCESS" : "FAILED");
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to change joint state");
    move_group_interface.execute(my_plan);
#endif

    geometry_msgs::Pose init_pose1;
    tf2::Quaternion q1;
    q1.setRPY(-M_PI/2, M_PI/2, M_PI);
    init_pose1.orientation.w = q1.getW();
    init_pose1.orientation.x = q1.getX();
    init_pose1.orientation.y = q1.getY();
    init_pose1.orientation.z = q1.getZ();

    init_pose1.position.x = 0.375,  
    init_pose1.position.y = -0.270;  
    init_pose1.position.z = 0.755; 

    
    visual_tools.deleteAllMarkers();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to PLAN");
    move_group_interface.setPoseTarget(init_pose1);

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("move_arm", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.publishAxisLabeled(init_pose1, "start_pose");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("right_arm_gripper_link"), joint_model_group, rvt::LIME_GREEN);
    visual_tools.trigger();

    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to EXECUTE");
    move_group_interface.execute(my_plan);
    return 0;
}

