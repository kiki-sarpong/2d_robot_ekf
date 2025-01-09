#ifndef _EKF_H_
#define _EKF_H_
#include <Eigen/Dense>


/* Struct for radar data. */
struct radar {
    double range;
    double azimuth;
    double velocity;  

    // Constructor to initialize the vector
    radar(double range_, double azimuth_, double velocity_)
        : range(range_), azimuth(azimuth_), velocity(velocity_) {}
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

    // Constructor to initialize the vector
    robot_vector(double x_val, double y_val, double theta_val)
        : x(x_val), y(y_val), theta_radians(theta_val) {}
};

/* Class for robot utilites */
class RobotUtility {
    public:
        double randomGaussian(double mean, double std);
        double distance(double x1, double y1, double x2, double y2);

};


class EKF {
public:
    // Constructor for the EKF class
    EKF();
    ~EKF();
    void calc_jacobian();
    void predict();
    void update();

private:
    double timestamp_ = 0;  // Time of the last measurement
    double dt;              // Delta time (time difference between updates)

    Eigen::VectorXd x;             // State vector
    Eigen::MatrixXd F;             // State transition matrix
    Eigen::MatrixXd P;             // State covariance matrix
    Eigen::MatrixXd Q;             // Process noise covariance matrix
    Eigen::MatrixXd R_radar;       // Measurement noise matrix for lidar
    Eigen::MatrixXd R_lidar;        // Measurement noise matrix for radar
    Eigen::MatrixXd H_lidar;        // Measurement update matrix for lidar
    Eigen::MatrixXd H_radar;        // Measurement update matrix for radar
    Eigen::MatrixXd jacobian_radar;


    float radar_noise = 0.5f;
    float lidar_noise = 0.08f;
    float p_noise = 3.0f;
    float q_noise = 0.4f;
};

#endif /* EKF_H_*/