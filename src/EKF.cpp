#include <iostream>
#include "EKF.h"
#include <glog/logging.h>


EKF::EKF() {
        // Initialize state vector (x)
        x = Eigen::VectorXd(6);
        x << 0, 0, 0, 0, 0, 0;

        // Initialize state transition matrix (F)
        F = Eigen::MatrixXd(6, 6);
        F << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

        // Initialize covariance matrix (P)
        P = Eigen::MatrixXd(6, 6);
        P << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

        // Initialize process noise covariance (Q)
        Q = Eigen::MatrixXd(6, 6);
        Q << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;
        
        // Initialize lidar noise matrix (R_lidar)
        R_lidar = Eigen::MatrixXd(2, 2);
        R_lidar << 1, 0,
                   0, 1;

        // Initialize lidar noise matrix (R_radar)
        R_radar = Eigen::MatrixXd(3, 3);
        R_radar <<  1, 0, 0, 
                    0, 0.1, 0, 
                    0, 0,  1;

        // Initialize measurement matrices for lidar and radar
        H_lidar = Eigen::MatrixXd(2, 6);
        H_lidar << 1, 0, 0, 0, 0, 0,
                   0, 1, 0, 0, 0, 0;

        H_radar = Eigen::MatrixXd(3, 6);
        H_radar << 1, 0, 0, 0, 0, 0,
                   0, 1, 0, 0, 0, 0,
                   0, 0, 1, 0, 0, 0;

        z_lidar = Eigen::VectorXd(2);
        z_radar = Eigen::VectorXd(3);;

        identity = Eigen::MatrixXd::Identity(6, 6); 

        P = P * p_noise;
        Q = Q * q_noise;
        R_lidar = R_lidar * lidar_noise;
        R_radar = R_radar * radar_noise;
}

EKF::~EKF(){}

void EKF::calc_jacobian(std::string sensor){
    if(sensor == "radar"){
        // Linearized model for radar system
        float c1 = x(0) * x(0) + x(1) * x(1); // x * x + y * y
        float c2 = sqrt(c1);   // sqrt(x^2 + y^2)
        float c3 = c1 * c2;     // qrt(x^2 + y^2)^3

        // Error handling for c1 so it never equals zero
        if (c1 == 0.0){LOG(ERROR) << "Zero division error in jacobian calculation. Skipping..\n"; return;}

        H_radar(0, 0) = x(0)/c2; // x/c2
        H_radar(0, 1) = x(1)/c2; // y/c2
        H_radar(1, 0) = -(x(1)/c1);  // -(y/c1)
        H_radar(1, 1) = x(0)/c1; // x/c1
        H_radar(2, 0) = x(3)/c2 - (x(1) * (x(3) * x(1) - x(4) * x(0)))/c3;  // (y * (vx * y - vy * x)))/c3
        H_radar(2, 1) = x(4)/c2 - (x(0) * (x(4) * x(0) - x(3) * x(1)))/c3;  // (x * (vy * x - vx * y)))/c3
        H_radar(2, 3) = x(0)/c2; // x/c2
        H_radar(2, 4) = x(1)/c2; // y/c2
        return;
    }
    // Linearized state model (After calculating the jacobian)
    // Updated state transition matrix
    F(0, 2) = -x(3) * dt * sin(x(2));  // -v_x * dt * sin(theta)
    F(0, 3) = dt * cos(x(3));       // dt * cos(theta)
    F(1, 2) = x(3) * dt * cos(x(2));  // v_x * dt * cos(theta)
    F(1, 3) = dt * sin(x(3));       // dt * sin(theta)
    F(2, 5) = dt;
}

void EKF::predict() {
    EKF::calc_jacobian();
    x = F * x;  // State prediction
    P = F * P * F.transpose() + Q;  // Covariance prediction
}

void EKF::update(std::string sensor) {
    // Update step
    if(sensor == "radar"){
        EKF::calc_jacobian(sensor);  // Calculate jacobian for radar
        K = P * H_radar.transpose() * (H_radar * P * H_radar.transpose() + R_radar).inverse();
        x = x + K * (z_radar - H_radar * x);
        P = (identity - K * H_radar) * P;
        return;
    }
    // Else run lidar
    K = P * H_lidar.transpose() * (H_lidar * P * H_lidar.transpose() + R_lidar).inverse();
    x = x + K * (z_lidar - H_lidar * x);
    P = (identity - K * H_lidar) * P;
}