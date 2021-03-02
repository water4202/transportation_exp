#ifndef UKF_H
#define UKF_H
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>

class ukf
{
public:
    ukf( int state_size , int measurement_size);
    ~ukf();

    float L;
    float dt;
    //vector size
    int x_size;
    int y_size;
    int x_sigmavector_size;
    int y_sigmavector_size;
    virtual Eigen::MatrixXd  dynamics(Eigen::MatrixXd  sigma_state);
    Eigen::MatrixXd rotate(float roll , float yaw , float pitch);

    void set_process_noise(Eigen::MatrixXd matrix);
    void set_measurement_noise(Eigen::MatrixXd matrix);
    void set_covariance_matrix(Eigen::MatrixXd matrix);

    void set_measurement_matrix(Eigen::MatrixXd matrix);
    void set_parameter(float alpha,float beta , float lambda , float kappa);

    void predict();
    void correct(Eigen::VectorXd measure);

    Eigen::VectorXd x ; //states
    Eigen::VectorXd x_a;
    Eigen::VectorXd x_a_hat;
    Eigen::VectorXd y ; //measurements

    Eigen::VectorXd x_hat; //x mean
    Eigen::VectorXd y_hat; //y mean

    float alpha ;
    float kappa ;
    float beta ;
    float lambda ;
private:

    Eigen::VectorXd w_c ; //weight c
    Eigen::VectorXd w_m ;  //weight m

    Eigen::MatrixXd x_a_sigmavector;
    Eigen::MatrixXd x_sigmavector ;
    Eigen::MatrixXd y_sigmavector ;
    Eigen::MatrixXd H ;    //measurement transform

    Eigen::MatrixXd P ; //covariance matrix

    Eigen::MatrixXd Q ; //noise matrix
    Eigen::MatrixXd R ; //noise matrix

    Eigen::MatrixXd P_a ; //covariance matrix
    Eigen::MatrixXd P_ ; //covariance matrix
    Eigen::MatrixXd P_yy ;
    Eigen::MatrixXd P_xy ;

    Eigen::MatrixXd Kalman_gain ;

    // float y_hat = 0.0;
    // float x_hat = 0.0;

};

#endif // UKF_H
