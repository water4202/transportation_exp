#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <mav_msgs/eigen_mav_msgs.h>

#define PI 3.1415926

Eigen::Matrix3d rotation_matrix;
sensor_msgs::Imu imu_data;
double payload_roll, payload_yaw, payload_pitch;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_data = *msg;

    double quaternion_w, quaternion_x, quaternion_y, quaternion_z;
    quaternion_x = imu_data.orientation.x;
    quaternion_y = imu_data.orientation.y;
    quaternion_z = imu_data.orientation.z;
    quaternion_w = imu_data.orientation.w;
    tf::Quaternion quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
    tf::Matrix3x3(quaternion).getRPY(payload_roll, payload_pitch, payload_yaw);

    double angular_velocity_x,angular_velocity_y,angular_velocity_z;
    angular_velocity_x = imu_data.angular_velocity.x;
    angular_velocity_y = imu_data.angular_velocity.y;
    angular_velocity_z = imu_data.angular_velocity.z;

    double linear_acceleration_x,linear_acceleration_y,linear_acceleration_z;
    linear_acceleration_x = imu_data.linear_acceleration.x;
    linear_acceleration_y = imu_data.linear_acceleration.y;
    linear_acceleration_z = imu_data.linear_acceleration.z;


    ROS_INFO("payload yaw: [%f]\n\n", payload_yaw * 180 / PI);

    printf("payload angular velocity\n");
    printf("x : %f\n", angular_velocity_x);
    printf("y : %f\n", angular_velocity_y);
    printf("z : %f\n\n", angular_velocity_z);

    printf("payload linear acceleration\n");
    printf("x : %f\n", linear_acceleration_x);
    printf("y : %f\n", linear_acceleration_y);
    printf("z : %f\n\n", linear_acceleration_z);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/mavros/imu/data", 1000, imuCallback);

    ros::spin();

    return 0;
}
