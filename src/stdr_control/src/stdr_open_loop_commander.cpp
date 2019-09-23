#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>



const double MINDistance = 1; //the robot would know if there is anything in front of robot within 1m
bool laser_alarm = false;
double minA = 0.0; //start angle of the scan
double maxA = 0.0; //end angle of the scan
double Aincrement = 0.0; // angular distance between measurements
double minrange = 0.0; //min range value
double maxrange = 0.0;//max range value
int ping_index = -1;
double upper_bound = 0.0;
double lower_bound = 0.0;
float front_distance = 2.0; // the front length of a lidar ping
float distance = 2.0; //distance detected by pings

void laserCallback(const sensor_msgs::LaserScan& laser_scan){
    if (ping_index<0){
        minA = laser_scan.angle_min;
        maxA = laser_scan.angle_max;
        Aincrement = laser_scan.angle_increment;
        minrange = laser_scan.range_min;
        maxrange = laser_scan.range_max;
        ping_index = (int) ((0.0-minA)/Aincrement);
}
 
front_distance = laser_scan.ranges[ping_index];
ROS_INFO("front distance is %f",front_distance);


upper_bound = (int) ping_index + 1 / Aincrement;
lower_bound = (int) ping_index - 1 / Aincrement;
laser_alarm = false;
for (int n = lower_bound; n < upper_bound; n++){

    distance = laser_scan.ranges[n]; // the distance from wall to current location
    if (distance < MINDistance){ // report if the distance from wall to current location is less than the MINDistance
       laser_alarm = true;
       ROS_WARN("Collision might happen");
    }
  }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "stdr_commander");
    ros::NodeHandle n;
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist >("/robot0/cmd_vel",1) ;
    ros::Subscriber lidar_subscriber = n.subscribe("/robot0/laser_1",1,laserCallback);


double sample_dt = 0.01; // specify a sample period of 10 ms
double speed = 2; // 1 m / s speed command
double yaw_rate = 0.5; // 0.5 rad / sec yaw rate command
double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds

geometry_msgs::Twist twist_cmd;

twist_cmd.linear.x =0.0;
twist_cmd.linear.y =0.0;
twist_cmd.linear.z =0.0;
twist_cmd.angular.x =0.0;
twist_cmd.angular.y =0.0;
twist_cmd.angular.z =0.0;

ros::Rate loop_timer(1/sample_dt);
double timer =0.0;
for (int i=0; i<10; i++){
  twist_commander.publish(twist_cmd);
  ros::spinOnce();
  loop_timer.sleep();
  }
while(ros::ok()){
     twist_cmd.angular.z=0.0;
     twist_cmd.linear.x=speed;
     while(!laser_alarm){
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
        ros::spinOnce();
        loop_timer.sleep();
       }
       twist_cmd.linear.x=0.0;
       twist_cmd.angular.z=yaw_rate;
       timer = 0.0;
       while(laser_alarm){
            twist_commander.publish(twist_cmd);
            timer+=sample_dt;
            ros::spinOnce();
            loop_timer.sleep();
       } 
    }
}
  
        
 
   


