#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <glog/logging.h>

#include "visualizatiion.h"
#include "EKF.h"

int main(int argc, char* argv[])
{
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();
    // Initialize glog 
    google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::INFO, "./../logs/mylog_");
    google::SetStderrLogging(google::INFO);  // Only ouput INFO or higher
    
    // Trajectory generation
    int amplitude = 10;  // Amplitude of sine wave
    int num_of_positions = 120; // Number of total points
    int y_offset = amplitude * 2; // Offset in the Y plane is twice the amplitude
    int start_point = 0.0; // Starting x location for trajectory
    int x_max = 50;   // Max X distance
    int y_max = 25 + amplitude; // Max Y distance
    
    LOG(INFO) << "Number of robot positions is " << num_of_positions;

    // Set noise values
    double lidar_noise = 5.0;
    double range_noise = 3.0, azimuth_noise = 0.5, velocity_noise = 2.0;
    double timestamp_noise = 10.0; // High variance in timestamps
    // 
    std::vector<lidar> lidar_data;
    std::vector<radar> radar_data;
    std::vector<robot_vector> robot_position;
    std::vector<double> timestamp_data;

    // Create a vector with values from 0 to pi divived into 200 uniform intervals for the x-axis
    sciplot::Vec x_vectors = sciplot::linspace(start_point, x_max, num_of_positions);
    
    // Initialize robot vector values
    double theta_radians = 0.0, y = 0.0, x = 0.0;
    double azimuth_radians = 0.0, range = 0.0, velocity = 0.0;
    
    // Initialize the update variable, set the constant time plus noise variable for the timestamp
    double time_stamp_sec = 0.0, time_const_sec = 10.0;
    
    // Initialize utility class
    RobotUtility robot_utlity;
    
    // Generating data
    for(int i=0; i<x_vectors.size(); i++){
        x = x_vectors[i];  // new x
        y = amplitude * std::sin(x) + y_offset; // new y

        // Generate timestamps
        time_stamp_sec += time_const_sec + robot_utlity.randomGaussian(timestamp_noise, timestamp_noise/3);
        // Generate lidar data
        double l_noise = robot_utlity.randomGaussian(0, lidar_noise);   // Mean and standard deviation
        float lidar_x = x + l_noise;
        float lidar_y = y + l_noise;
        if (i > 0){
            // Made this assignment for easier readability
            double& x1 = robot_position[i-1].x;
            double& y1 = robot_position[i-1].y;
            double& x2 = x;
            double& y2 = y;

            // Get angle of robot
            double theta_radians = (atan2((y2-y1), (x2-x1)) + M_PI/2);
            // Correct angle value is the theta_radians is less than zero
            theta_radians = (theta_radians < 0) ? theta_radians +=2*M_PI : theta_radians;
            
            // Generate radar data
            double r_noise = robot_utlity.randomGaussian(0, range_noise);   // Mean and standard deviation
            float distance = robot_utlity.distance(x1, y1, x2, y2);
            range = distance + r_noise; // Add noise to distance
            // Add noise to azimuth
            azimuth_radians = theta_radians + robot_utlity.randomGaussian(0, azimuth_noise);
            azimuth_radians = std::fmod(azimuth_radians, (2*M_PI));  // azimuth is in radians and should be b/n 0 - 2PI
            // Calculate velocity
            velocity = distance/time_stamp_sec + robot_utlity.randomGaussian(velocity_noise, velocity_noise/4);

            // Convert to degrees
            // std:: cout << theta_radians * 180/M_PI<< "  \n";   
            // std::cout << azimuth_radians << "\n";
        }
        
        // Save data
        robot_position.emplace_back(robot_vector(x, y, theta_radians));  // Save the robot positions
        timestamp_data.emplace_back(time_stamp_sec);     // Save the timestamps
        lidar_data.emplace_back(lidar(lidar_x, lidar_y));
        radar_data.emplace_back(radar(range, azimuth_radians, velocity));


        // Main EKF pipeline
        EKF ekf_model;  // Create instance of EKF model
        ekf_model.x << robot_position[0].x, robot_position[0].y, robot_position[0].theta_radians, 0, 0, 0;
        double dt = timestamp_data[1] - timestamp_data[0];

        for(int i=1; i < timestamp_data.size(); i++){
            if (i > 1){
                dt = timestamp_data[i] - ekf_model.timestamp_;
            }
            ekf_model.predict();
            // Lidar update
            ekf_model.z_lidar << lidar_data[i].x, lidar_data[i].y;
            // Radar update
            ekf_model.z_radar << radar_data[i].range, radar_data[i].azimuth, radar_data[i].velocity;

            ekf_model.update("lidar");

            ekf_model.timestamp_ = timestamp_data[i];
        }

        // Convert to radians 
        // LOG(INFO) << x  << "  " << y << "  "<< theta_radians  * 180/M_PI << " degrees \n";
    }

    // Visualize viz(x_max, y_max);
    // viz.display_graph(x, amplitude, y_offset, false);

    // Complete timer
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    LOG(INFO) << "Program took " << duration.count() << " seconds to complete.";
}
