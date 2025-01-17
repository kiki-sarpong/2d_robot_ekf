#ifndef _EKF_H_
#define _EKF_H_

#include <Eigen/Dense>


/* Struct for radar data. */
struct radar {
    double range;
    double azimuth;
    double radial_velocity;  

    // Constructor to initialize the vector
    radar(double range_, double azimuth_, double velocity_radial)
        : range(range_), azimuth(azimuth_), radial_velocity(velocity_radial) {}
};

/* Struct for lidar data. */
struct lidar {
    double x;
    double y;

    // Constructor to initialize the vector
    lidar(double x_val, double y_val): x(x_val), y(y_val) {}
};

/* Struct for the robot vector coordinates. */
struct robot_vector {
    double x;
    double y;
    double theta_radians;  // Angle in radians
    double velocity_x;
    double velocity_y;

    // Constructor to initialize the vector
    robot_vector(double x_val, double y_val, double theta_val, double velocity__x, double velocity__y)
        : x(x_val), y(y_val), theta_radians(theta_val), velocity_x(velocity__x), velocity_y(velocity__y) {}
};

/* Class for robot utilites */
class RobotUtility {
    public:
        double randomGaussian(double mean, double std);
        double distance(double x1, double y1, double x2, double y2);

};

/*
Class for Extended kalman filter 
*/
class EKF {
    public:
        // Constructor for the EKF class
        EKF();
        ~EKF();
        double timestamp_ = 0;  // Time of the last measurement
        double dt = 0;          // Delta time (time difference between updates)
        Eigen::VectorXd z_lidar;
        Eigen::VectorXd z_radar;
        Eigen::VectorXd x;             // State vector
        Eigen::MatrixXd P;             // State covariance matrix
        void calc_jacobian(std::string sensor="none");
        void predict();
        void update(std::string sensor);

    private:
        Eigen::MatrixXd F;             // State transition matrix
        Eigen::MatrixXd Q;             // Process noise covariance matrix
        Eigen::MatrixXd R_radar;       // Measurement noise matrix for lidar
        Eigen::MatrixXd R_lidar;        // Measurement noise matrix for radar
        Eigen::MatrixXd H_lidar;        // Measurement update matrix for lidar
        Eigen::MatrixXd H_radar;        // Measurement update matrix for radar
        Eigen::MatrixXd K;          // Kalman gain
        Eigen::MatrixXd identity;

        float radar_noise = 0.001f; //0.3f;
        float lidar_noise = 0.001f;  //0.3f;
        float p_noise = 0.001f;  // 0.5f;
        float q_noise = 0.001f;  //0.1f;
};

#endif /* EKF_H_*/