#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <string>

//prottype;
class getAndSend{
    public:
    getAndSend(){
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

        distance_sub = n.subscribe("/camera/yolo/distance", 1000, &getAndSend::velocity_Callback, this);
        direction_sub = n.subscribe("/camera/yolo/direction", 1000, &getAndSend::angular_Callback, this);
    }
    
    void velocity_Callback(const std_msgs::String::ConstPtr &msg)
        {
            
            geometry_msgs::Twist cmd_vel;
            try
            {
                double distance = std::stod(msg->data.c_str());
                if (distance <= 1200 && distance > 100)
                {
                    velocity = -0.3;
                }
                else if (distance > 2000)
                {
                    velocity = 0.3;
                }else {
                    velocity = 0;     
                }
            }
            catch (const std::exception& e)
            {
                std::cout << e.what() << std::endl;
            }
            //ROS_INFO(cmd_vel.linear.x);
            // cmd_vel_pub.publish(cmd_vel);
        }
    void angular_Callback(const std_msgs::String::ConstPtr &msg)
        {
            
            geometry_msgs::Twist cmd_vel;
            try
            {
                int16_t capability = 240*0.2;
                double direction = std::stod(msg->data.c_str());
                if (direction < -capability )
                {
                    cmd_vel.angular.z = -0.3;
                }
                else if (direction >capability)
                {
                    cmd_vel.angular.z = 0.3;
                }else {
                    cmd_vel.angular.z = 0;     
                }
            }
            catch (const std::exception& e)
            {
                std::cout << e.what() << std::endl;
            }
            if(velocity <0){
                cmd_vel.angular.z *= -1; 
            }
            //ROS_INFO(cmd_vel.linear.x);
            cmd_vel.linear.x = velocity;
            cmd_vel_pub.publish(cmd_vel);
        }
    private:
        ros::NodeHandle n;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber distance_sub, direction_sub;
        int16_t velocity = 0;
        int16_t angular_v = 0;
};






 
int main(int argc, char **argv)
{
    //initialize
    ros::init(argc, argv, "cmd_to_zumo");
    getAndSend GAS;

    //ros::NodeHandle n;
    ros::spin();

    return 0;
}
