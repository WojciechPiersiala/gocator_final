#include <ros/ros.h>


#include <termios.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <cmath>

#include <std_msgs/String.h>

#define START ('1')
#define STOP  ('2')
#define SEND  ('3')
#define PERS  ('4')
#define SAVE  ('5')

char getch();
int main(int argc, char **argv) {
    ros::init(argc, argv, "set_speed");
    ros::NodeHandle nh;
	ros::Publisher command_pub;
	command_pub = nh.advertise<std_msgs::String>("scan_command",1);

	std_msgs::String out_command;


	char key, action;
	ROS_INFO_STREAM("started");	
	while(ros::ok()){
		if(key = getch()){
            out_command.data = key;
			command_pub.publish(out_command);
			ROS_INFO_STREAM(out_command.data);	
		}
	}
	return 0;
}




char getch(){
		fd_set set;
		struct timeval timeout;
		int rv;
		char buff = 0;
		int len = 1;
		int filedesc = 0;
		FD_ZERO(&set);
		FD_SET(filedesc, &set);
		
		timeout.tv_sec = 0;
		timeout.tv_usec = 1000;

		rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

		struct termios old = {0};
		if (tcgetattr(filedesc, &old) < 0)
			ROS_ERROR("tcsetattr()");
		old.c_lflag &= ~ICANON;
		old.c_lflag &= ~ECHO;
		old.c_cc[VMIN] = 1;
		old.c_cc[VTIME] = 0;
		if (tcsetattr(filedesc, TCSANOW, &old) < 0)
			ROS_ERROR("tcsetattr ICANON");

		if(rv == -1)
			ROS_ERROR("select");
		else if(rv == 0)
			bool nothing = false;
		else
			read(filedesc, &buff, len );

		old.c_lflag |= ICANON;
		old.c_lflag |= ECHO;
		if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
			ROS_ERROR ("tcsetattr ~ICANON");
		return (buff);
	}