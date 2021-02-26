#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>

#define PI 3.1415926
bool yaw_flag = true;
bool imu_flag = false;

void optitrack_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped optitrack_data;
    optitrack_data = *msg;
    while(yaw_flag == true){
        double quaternion_w, quaternion_x, quaternion_y, quaternion_z;
        double payload_roll, payload_yaw, payload_pitch;
        quaternion_x = optitrack_data.pose.orientation.x;
        quaternion_y = optitrack_data.pose.orientation.y;
        quaternion_z = optitrack_data.pose.orientation.z;
        quaternion_w = optitrack_data.pose.orientation.w;
        tf::Quaternion quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
        tf::Matrix3x3(quaternion).getRPY(payload_roll, payload_pitch, payload_yaw);

        printf("payload yaw: [%f]\n\n", payload_yaw * 180 / PI);
        yaw_flag = false;
        imu_flag = true;
    }

}

void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu imu_data;
    imu_data = *msg;
    while(imu_flag == true){
        double angular_velocity_x,angular_velocity_y,angular_velocity_z;
        angular_velocity_x = imu_data.angular_velocity.x;
        angular_velocity_y = imu_data.angular_velocity.y;
        angular_velocity_z = imu_data.angular_velocity.z;

        double linear_acceleration_x,linear_acceleration_y,linear_acceleration_z;
        linear_acceleration_x = imu_data.linear_acceleration.x;
        linear_acceleration_y = imu_data.linear_acceleration.y;
        linear_acceleration_z = imu_data.linear_acceleration.z;

        printf("payload angular velocity\n");
        printf("x : %f\n", angular_velocity_x);
        printf("y : %f\n", angular_velocity_y);
        printf("z : %f\n\n", angular_velocity_z);

        printf("payload linear acceleration\n");
        printf("x : %f\n", linear_acceleration_x);
        printf("y : %f\n", linear_acceleration_y);
        printf("z : %f\n\n", linear_acceleration_z);

        yaw_flag = true;
        imu_flag = false;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh;

    ros::Subscriber yaw_sub = nh.subscribe("/vrpn_client_node/RigidBody7/pose", 1000, optitrack_Callback);
    ros::Subscriber acc_angular_vel_sub = nh.subscribe("/mavros/imu/data", 1000, imu_Callback);

    ros::spin();

    return 0;
}
