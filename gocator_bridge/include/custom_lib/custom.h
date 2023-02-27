#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <termios.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace custom{
	
	char getch();
	float HSVtoRGB(float H, float S,float V);
	void transform(geometry_msgs::Point32 &p, const Eigen::Matrix3f &R, const geometry_msgs::Point32 & T);
	void transform2(pcl::PointXYZRGB &p,       const Eigen::Matrix3f &R, const geometry_msgs::Point32 & T);

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
			// ROS_INFO("no_key_pressed");
			bool nothing = false;
			// return ' ';
		else
			read(filedesc, &buff, len );

		old.c_lflag |= ICANON;
		old.c_lflag |= ECHO;
		if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
			ROS_ERROR ("tcsetattr ~ICANON");
		return (buff);
	}


	float HSVtoRGB(float H, float S,float V){
		if(H>360 || H<0 || S>100 || S<0 || V>100 || V<0){
			// std::cout<<"The givem HSV values are not in valid range"<<std::endl;
			return 0;
		}
		float s = S/100;
		float v = V/100;
		float C = s*v;
		float X = C*(1-abs(fmod(H/60.0, 2)-1));
		float m = v-C;
		float r,g,b;
		if(H >= 0 && H < 60){
			r = C,g = X,b = 0;
		}
		else if(H >= 60 && H < 120){
			r = X,g = C,b = 0;
		}
		else if(H >= 120 && H < 180){
			r = 0,g = C,b = X;
		}
		else if(H >= 180 && H < 240){
			r = 0,g = X,b = C;
		}
		else if(H >= 240 && H < 300){
			r = X,g = 0,b = C;
		}
		else{
			r = C,g = 0,b = X;
		}

		uint8_t R = (r+m)*255;
		uint8_t G = (g+m)*255;
		uint8_t B = (b+m)*255;

		uint32_t RGB;
		RGB = static_cast<uint32_t>(R) << 16 | static_cast<uint32_t>(G) << 8 | static_cast<uint32_t>(B);
		float res = *reinterpret_cast<float*>(&RGB);
		return res;
	}

	void transform(geometry_msgs::Point32 &p, const Eigen::Matrix3f &R, const geometry_msgs::Point32 & T){
		// std::cout << "Transform started " <<std::endl;
		Eigen::Vector3f v, v_out;
		p.x += T.x;
		p.y += T.y;
		p.z += T.z;

		v << p.x, p.y, p.z;
		v_out = R*v;

		// std::cout <<" v: " << v <<std::endl;
		// std::cout <<" R: " << R <<std::endl;
		// std::cout << "V_out :" <<v_out <<std::endl;

		p.x =v_out[0];
		p.y =v_out[1];
		p.z =v_out[2];
	}

		void transform2(pcl::PointXYZRGB &p, const Eigen::Matrix3f &R, const geometry_msgs::Point32 & T){
		// std::cout << "Transform started " <<std::endl;
		Eigen::Vector3f v, v_out;
		p.x += T.x;
		p.y += T.y;
		p.z += T.z;

		v << p.x, p.y, p.z;
		v_out = R*v;

		// std::cout <<" v: " << v <<std::endl;
		// std::cout <<" R: " << R <<std::endl;
		// std::cout << "V_out :" <<v_out <<std::endl;

		p.x =v_out[0];
		p.y =v_out[1];
		p.z =v_out[2];
	}
}

#endif //KEYBOARD_H