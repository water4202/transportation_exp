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

#define normal
#define PI 3.1415926

double k1 = 1.0, k2 = 2.0, k3 = 1.0, kv = 2.5, kw = 7.0;
double mp = 0.5, L = 1.0, g = 9.8, Izz = mp*L*L/12;

Eigen::Vector3d pose, vel, ang;
Eigen::Vector3d v_p;
Eigen::Vector3d FL_des;
Eigen::Vector3d r_p_c2(-0.5, 0, 0);
double vir_x, vir_y, theta_r, vx, vy, ax, ay, jx, jy;
double last_w = 0.0;

double payload_roll, payload_yaw, payload_pitch;
Eigen::Vector3d v_w_eta;
Eigen::Vector3d pc2_est;

double x_e, y_e, theta_e;
Eigen::Vector3d err_state;
unsigned int tick=0;
bool flag = false;

Eigen::Matrix3d R_pl_B;
Eigen::Matrix3d uav_rotation;
Eigen::Vector3d w_;

geometry_msgs::PoseStamped desired_pose;
geometry_msgs::Point record_pose;
geometry_msgs::Point desired_force;
geometry_msgs::Point desired_velocity;
geometry_msgs::Point feedforward;

sensor_msgs::Imu imu_data;
void payload_imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;

  double w,x,y,z;
  x = imu_data.orientation.x;
  y = imu_data.orientation.y;
  z = imu_data.orientation.z;
  w = imu_data.orientation.w;
  tf::Quaternion Q(x, y, z, w);
  tf::Matrix3x3(Q).getRPY(payload_roll, payload_pitch, payload_yaw);

  R_pl_B << cos(payload_yaw), sin(payload_yaw),   0,
           -sin(payload_yaw), cos(payload_yaw),   0,
                           0,                0,   1;

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

  ros::Publisher feedforward_pub = nh.advertise<geometry_msgs::Point>("/feedforward",2);
  ros::Publisher desired_pose_pub = nh.advertise<geometry_msgs::Point>("/drone1/desired_position",2);
  ros::Publisher desired_force_pub = nh.advertise<geometry_msgs::Point>("/desired_force",2);
  ros::Publisher desired_velocity_pub = nh.advertise<geometry_msgs::Point>("/desired_velocity",2);
  ros::Publisher traj_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly1/command/pose",2);

  ros::Subscriber imu1_sub = nh.subscribe("/mavros/imu/data",2,payload_imu_callback);
  ros::Subscriber est_vel_sub = nh.subscribe<geometry_msgs::Point>("est_vel",3,est_vel_cb);
  ros::Subscriber pc2_sub = nh.subscribe("pointpc2",2,pc2_cb);
  ros::Subscriber eta_sub = nh.subscribe("pointvc2",2,eta_cb);

  ros::Rate loop_rate(50.0);
  nh.setParam("/start",false);
  geometry_msgs::PoseStamped force;

  //planning
  qptrajectory plan;
  path_def path;
  trajectory_profile p1,p2,p3,p4,p5,p6,p7,p8;
  std::vector<trajectory_profile> data;

    p1.pos << 1,1,0;
    p1.vel << 0,0,0;
    p1.acc << 0,0,0;
    p1.yaw = 0;

    p2.pos << 3,5,0;
    p2.vel << 0,0,0;
    p2.acc << 0,0,0;
    p2.yaw = 0;

    p3.pos << 12,0,0;
    p3.vel << 0,0,0;
    p3.acc << 0,0,0;
    p3.yaw = 0;

    p4.pos << 3,-5,0;
    p4.vel << 0,0,0;
    p4.acc << 0,0,0;
    p4.yaw = 0;

    p5.pos << -3,5,0;
    p5.vel << 0,0,0;
    p5.acc << 0,0,0;
    p5.yaw = 0;

    p6.pos << -12,0,0;
    p6.vel << 0,0,0;
    p6.acc << 0,0,0;
    p6.yaw = 0;

    p7.pos << -3,-5,0;
    p7.vel << 0,0,0;
    p7.acc << 0,0,0;
    p7.yaw = 0;

    p8.pos << 0,0,0;
    p8.vel << 0,0,0;
    p8.acc << 0,0,0;
    p8.yaw = 0;

  path.push_back(segments(p1,p2,12.0));
  path.push_back(segments(p2,p3,16.0));
  path.push_back(segments(p3,p4,16.0));
  path.push_back(segments(p4,p5,16.0));
  path.push_back(segments(p5,p6,16.0));
  path.push_back(segments(p6,p7,16.0));
  path.push_back(segments(p7,p8,16.0));
  data = plan.get_profile(path,path.size(),0.02);

  desired_pose.pose.position.x = 0.9;
  desired_pose.pose.position.y = 0.0;
  desired_pose.pose.position.z = 1.3;

  while(ros::ok()){

    vir_x = data[tick].pos(0);
    vir_y = data[tick].pos(1);
    record_pose.x = vir_x;
    record_pose.y = vir_y;
    vx = data[tick].vel(0);
    vy = data[tick].vel(1);
    ax = data[tick].acc(0);
    ay = data[tick].acc(1);
    jx = data[tick].jerk(0);
    jy = data[tick].jerk(1);

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

    FL_des(0) = cmd_(0);   // + nonlinearterm(0);// + vd_dot ;
    FL_des(1) = cmd_(1);   // + w_d_dot;

    desired_force.x = FL_des(0);
    desired_force.y = FL_des(1);

    desired_velocity.x = vr;
    desired_velocity.y = w_r; //put w into y

    feedforward.x = nonlinearterm(0);
    feedforward.y = vd_dot;
    feedforward.z = w_d_dot;

    force.pose.position.x = FL_des(0);
    /* use "position" element to store force message -> lee_position_controller.cpp line 133
       *acceleration =  -command_trajectory_.position_W/ vehicle_parameters_.mass_
       - vehicle_parameters_.gravity_ * e_3 + Eigen::Vector3d(0,0,0) ;
    */
    force.pose.position.y = FL_des(1);

     traj_pub.publish(force);
     std::cout << force.pose.position.x << std::endl;
     desired_pose_pub.publish(record_pose);
     desired_force_pub.publish(desired_force);
     desired_velocity_pub.publish(desired_velocity);
     feedforward_pub.publish(feedforward);

     ros::spinOnce();
     loop_rate.sleep();
    }
}