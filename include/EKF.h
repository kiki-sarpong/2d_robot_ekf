#ifndef _EKF_H_
#define _EKF_H_


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


#endif /* EKF_H_*/