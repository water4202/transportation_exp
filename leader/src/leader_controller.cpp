#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <qptrajectory.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <tf2/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

#define PAYLOAD_LENGTH 1.58
#define normal
#define PI 3.1415926

double k1 = 1.0, k2 = 2.0, k3 = 1.0, kv = 2.5, kw = 7.0;
double mp = 0.5, L = 1.0, g = 9.8, Izz = mp*L*L/12;

Eigen::Vector3d pose, vel;
Eigen::Vector3d v_p;
Eigen::Vector3d r_p_c2(-0.5 * PAYLOAD_LENGTH, 0, 0);
double vir_x, vir_y, theta_r, vx, vy, ax, ay, jx, jy;
double last_w = 0.0;

float payload_yaw;
Eigen::Vector3d v_w_eta;
Eigen::Vector3d pc2_est;

double x_e, y_e, theta_e;
Eigen::Vector3d err_state;
unsigned int tick = 0;
bool flag = false;

Eigen::Matrix3d R_pl_B;
Eigen::Vector3d w_;

geometry_msgs::PoseStamped desired_pose;
geometry_msgs::Point controller_force;

geometry_msgs::Point debug_msg;

sensor_msgs::Imu imu_data;
void payload_imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;

  // double w,x,y,z;
  // x = imu_data.orientation.x;
  // y = imu_data.orientation.y;
  // z = imu_data.orientation.z;
  // w = imu_data.orientation.w;
  // tf::Quaternion Q(x, y, z, w);
  // tf::Matrix3x3(Q).getRPY(payload_roll, payload_pitch, payload_yaw);

  // R_pl_B << cos(payload_yaw), sin(payload_yaw),   0,
  //          -sin(payload_yaw), cos(payload_yaw),   0,
  //                          0,                0,   1;

  Eigen::Vector3d tmp;
  tmp << imu_data.angular_velocity.x,
         imu_data.angular_velocity.y,
         imu_data.angular_velocity.z;

  w_ << 0, 0, tmp(2);
}

void est_vel_cb(const geometry_msgs::Point::ConstPtr& msg){
  Eigen::Vector3d vc1 = Eigen::Vector3d(msg->x, msg->y, msg->z);
  v_p = R_pl_B*vc1;
}

void pc2_cb(const geometry_msgs::Point::ConstPtr& msg){
  pc2_est << msg->x, msg->y, msg->z;
}

void eta_cb(const geometry_msgs::Point::ConstPtr& msg){
  v_w_eta << msg->x, msg->y, msg->z;
}

void optitrack_payload_yaw_callback(const geometry_msgs::Pose2D::ConstPtr& msg){
  geometry_msgs::Pose2D optitrack_payload_data;
  optitrack_payload_data = *msg;
  payload_yaw = optitrack_payload_data.theta;
}

Eigen::Vector3d nonholonomic_output(double x_r, double y_r, double theta_r, double v_r, double w_r){

  Eigen::Vector3d output;
  Eigen::Vector3d err_state_B;
  err_state << x_r - pc2_est(0), y_r - pc2_est(1), theta_r - payload_yaw; // +(PI/2);(5)(6)
  err_state_B = R_pl_B * err_state;

  x_e = err_state_B(0);
  y_e = err_state_B(1);
  theta_e = err_state(2);

  double vd = v_r*cos(theta_e) + k1*x_e;   //(43) k1
  double w_d = w_r + v_r*k2*y_e + k3*sin(theta_e);    //k2 k3

  output << vd, w_d, 0;
  return output;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "leader_controller");
  ros::NodeHandle nh;

  ros::Subscriber imu1_sub = nh.subscribe("/mavros/imu/data",2,payload_imu_callback);
  //ros::Subscriber imu1_sub = nh.subscribe("/payload/IMU1",2,payload_imu_callback);
  ros::Subscriber est_vel_sub = nh.subscribe<geometry_msgs::Point>("est_vel",3,est_vel_cb);
  ros::Subscriber pc2_sub = nh.subscribe("pointpc2",2,pc2_cb);
  ros::Subscriber eta_sub = nh.subscribe("pointvc2",2,eta_cb);
  ros::Subscriber yaw_sub = nh.subscribe("optitrack_payload_yaw",2,optitrack_payload_yaw_callback);

  ros::Publisher controller_force_pub = nh.advertise<geometry_msgs::Point>("/controller_force",2);
  ros::Publisher debug_pub = nh.advertise<geometry_msgs::Point>("/debug_msg",2);

  ros::Rate loop_rate(50.0);
  //nh.setParam("/start",false);
  geometry_msgs::PoseStamped force;

  //planning
  qptrajectory plan;
  path_def path;
  trajectory_profile p1,p2,p3,p4,p5,p6,p7,p8;
  std::vector<trajectory_profile> data;

    p1.pos << 0,0,0;
    p1.vel << 0,0,0;
    p1.acc << 0,0,0;
    p1.yaw = 0;

    p2.pos << 0.25,-0.4,0;
    p2.vel << 0,0,0;
    p2.acc << 0,0,0;
    p2.yaw = 0;

    p3.pos << 1.2,0,0;
    p3.vel << 0,0,0;
    p3.acc << 0,0,0;
    p3.yaw = 0;

    p4.pos << 0.25,0.5,0;
    p4.vel << 0,0,0;
    p4.acc << 0,0,0;
    p4.yaw = 0;

    p5.pos << -0.4,-0.5,0;
    p5.vel << 0,0,0;
    p5.acc << 0,0,0;
    p5.yaw = 0;

    p6.pos << -1.2,0,0;
    p6.vel << 0,0,0;
    p6.acc << 0,0,0;
    p6.yaw = 0;

    p7.pos << -0.4,0.5,0;
    p7.vel << 0,0,0;
    p7.acc << 0,0,0;
    p7.yaw = 0;

    p8.pos << 0,0,0;
    p8.vel << 0,0,0;
    p8.acc << 0,0,0;
    p8.yaw = 0;

  path.push_back(segments(p1,p2,4.0));
  path.push_back(segments(p2,p3,4.0));
  path.push_back(segments(p3,p4,4.0));
  path.push_back(segments(p4,p5,4.0));
  path.push_back(segments(p5,p6,4.0));
  path.push_back(segments(p6,p7,4.0));
  path.push_back(segments(p7,p8,4.0));
  data = plan.get_profile(path,path.size(),0.02);

  desired_pose.pose.position.x = 0.05;
  desired_pose.pose.position.y = 0.0;
  desired_pose.pose.position.z = 0.6;


  while(ros::ok()){

    vir_x = data[tick].pos(0);
    vir_y = data[tick].pos(1);
    vx = data[tick].vel(0);
    vy = data[tick].vel(1);
    ax = data[tick].acc(0);
    ay = data[tick].acc(1);
    jx = data[tick].jerk(0);
    jy = data[tick].jerk(1);

    debug_msg.x = data[tick].pos(0);
    debug_msg.y = data[tick].vel(0);
    debug_msg.z = data[tick].acc(0);

    theta_r = atan2(data[tick].vel(1),data[tick].vel(0));   //(4)

    if(theta_r <0){
      theta_r += 2*PI;
    }

    Eigen::Vector3d alpha;
    alpha << 0, 0, (w_(2) - last_w)/0.02;
    last_w = w_(2); //payload imu

    double w_r = (ay*vx - ax*vy)/(vx*vx + vy*vy); //(theta_r - last_theta_r) /(0.02) ;
    double vr = sqrt(vx*vx + vy*vy);

    Eigen::Vector3d nonholoutput = nonholonomic_output(vir_x, vir_y, theta_r, vr, w_r);   //vd, w_d, 0
    double vr_dot = sqrt(ax*ax + ay*ay);
    double theta_e_dot = w_r - w_(2);  //the error of the angular velocity
    double x_e_dot = w_(2) * y_e + vr*cos(theta_e) - v_w_eta(0);  //(58)
                                                    //UKF 2nd
    double y_e_dot = - w_(2) * x_e + vr*sin(theta_e);  //(58)
    double w_r_dot = (jy*vx - jx*vy)/(vr*vr) - (2*vr_dot*w_r)/vr;     //vr^(-3) ??
    double w_d_dot = w_r_dot + vr_dot*k2*y_e + vr*k2*y_e_dot + k3*theta_e_dot*cos(theta_e);   //take (43) time derivative
    double vd_dot = vr_dot*cos(theta_e) - vr*theta_e_dot*sin(theta_e) + k1*x_e_dot;

    Eigen::Vector3d nonlinearterm;

    nonlinearterm = w_.cross(v_p) - alpha.cross(r_p_c2) - w_.cross(w_.cross(r_p_c2)); //the last term of (41)

    if( nonholoutput(0) > 10 ){   //vd
      nonholoutput(0) = 10;
    }

    Eigen::Vector3d tmp;
    Eigen::Vector3d cmd_;

    //(41)(42) separately
    tmp << kv * (nonholoutput(0) - v_w_eta(0)) + x_e + nonlinearterm(0) + vd_dot,
           kw * (nonholoutput(1) - v_w_eta(1)) + sin(theta_e)/k2 + w_d_dot,   //ffy is close to zero.
           0;

    Eigen::Matrix3d M;
    M <<   mp,        0,    0,
            0,  2*Izz/L,    0,
            0,        0,    1;

    cmd_ = R_pl_B.transpose() * M * tmp;

    tick++;


    controller_force.x = cmd_(0);   // + nonlinearterm(0);// + vd_dot ;
    controller_force.y = cmd_(1);   // + w_d_dot;

    controller_force_pub.publish(controller_force);
    debug_pub.publish(debug_msg);

    std::cout << "payload_yaw " << payload_yaw << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }
}
