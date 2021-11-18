#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <string>
#include <vector>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>


#define RANGE_MAX 5.6;
using namespace std;
//prottype;
class Lidar_detection{
    public:
    Lidar_detection(){
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        lidar_sub = n.subscribe("/scan", 1000, &Lidar_detection::lidar_callback, this);
        message_pub = n.advertise<std_msgs::String>("/scan/degree", 1000);

    }

    double meanWithoutInf(vector<double> vec){
        double result=0;
        int size=0;
        for (int i = 0; i < vec.size(); i++)
        {
            if(vec[i]<10){
                result += vec[i];
                size++;
            }
        }
        result = result/size;
        return result;
    }

    double null_check(double target){
        if(!(target >0)){
            target=(double)RANGE_MAX;
        }

        return target;
    }
    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
            geometry_msgs::Twist cmd_vel;
            std_msgs::String msg_data;
            std::stringstream ss;
            // string chatter;
            double center_number = (-msg->angle_min)/msg->angle_increment;
            double angle_min = (msg->angle_min)/msg->angle_increment;
            double angle_max = (-msg->angle_min)/msg->angle_increment;
            double center=msg->ranges[center_number+180];
            double left=msg->ranges[center_number+128];
            double right=msg->ranges[center_number-128];
            std::stringstream angles;
            vector<double> q1,q2, q3, q4, q5, q6,q7,q8;
            double min1=0;
            double min2=0;
            double min3=0; 
            double min4=0;
            // angle 0 - 180, -179 - 0
            for (double angle = angle_min; angle < angle_max; angle++)            
            {
                if(angle >=0 && angle<90){
                    q1.push_back(msg->ranges[center_number+angle]);
                }else if (angle >=90 && angle <angle_max){
                    q2.push_back(msg->ranges[center_number+angle]);
                }
                if (angle >=angle_min && angle <-90){
                    q3.push_back(msg->ranges[center_number+angle]);
                }else if (angle >=-90 && angle <0){
                    q4.push_back(msg->ranges[center_number+angle]);
                }

                // if (center_number+angle >=45 && center_number+angle <90){
                //     q1.push_back(msg->ranges[center_number+angle]);
                // }
                // q1.push_back(msg->ranges[center_number+angle]);
            }
            
             
            center=null_check(center);
            left=null_check(left);
            right=null_check(right);

            ROS_INFO("center:[%If], left[%If], right[%If]", center, left, right);
            ROS_INFO("center_number: [%If]", center_number);
            ss << "center: " << center << " left " << left << " right " << right << " center_number " << center_number;    
            

            try
            {
                // auto sm1 = min_element(q1.begin(), q1.end());
                // auto sm2 = min_element(q2.begin(), q2.end());
                // auto sm3 = min_element(q3.begin(), q3.end());
                // auto sm4 = min_element(q4.begin(), q4.end());
                // sort(q1.begin(), q1.end());
                // sort(q2.begin(), q2.end());
                // sort(q3.begin(), q3.end());
                // sort(q4.begin(), q4.end());
                min1 = meanWithoutInf(q1);
                min2 = meanWithoutInf(q2);
                min3 = meanWithoutInf(q3);
                min4 = meanWithoutInf(q4);
                // min1 = q1[q1.size()/2-1];
                // min2 = q2[q2.size()/2-1];
                // min3 = q3[q3.size()/2-1];
                // min4 = q4[q4.size()/2-1];
                angles << " left front: " << min1
                << "// " << "left back: " << min2
                << "// " << "right back: " << min3
                << "// " << "right front: " << min4;
            }
            catch(const std::exception& e)
            {

            }
            
            //left front
            if(min1<1){
                cmd_vel.linear.x = -0.2;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            //left back
            if(min2<1){
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.2;

            }
            //right back
            if(min3<1){
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = -0.2;
            }
            //right front
            if(min4<1){
                cmd_vel.linear.x = -0.2;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;

            }
            // chatter +="center ";
            // chatter += center;
            
            msg_data.data = angles.str();
            // msg_data.data = chatter.c_str();

            //ROS_INFO(cmd_vel.linear.x);
            message_pub.publish(msg_data);
            cmd_vel_pub.publish(cmd_vel);

        }
    private:
        ros::NodeHandle n;
        ros::Publisher cmd_vel_pub, message_pub;
        ros::Subscriber lidar_sub;
};


 
int main(int argc, char **argv)
{
    //initialize
    ros::init(argc, argv, "lidar_detection");
    Lidar_detection Lidar;

    //ros::NodeHandle n;
    ros::spin();

    return 0;
}

