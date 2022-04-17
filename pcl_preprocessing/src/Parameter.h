/**
 * @file Parameter.h
 * @author Tim Spilak (tspilak@stud.hs-heilbronn.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
// general
#include "ros/ros.h"

// math operations
#include <Eigen/Dense>
#include <cmath>

const double perc2rad(double percent);
const double degree2rad(double rad);
const double get_z_threshold(double length, double max_angle);

// outlier
const double radius = 0.1;                                              // radius for points [ m ]
const double min_neighbor = 1;                                          // minimum number of neighbors

// Parameter ROI
const double roi_width  = 10.0;                                         // width of ROI [ m ]
const double roi_length = 75.0;                                         // lenght of ROI [ m ]
const double roi_mid = 15;                                              // distance from rear end of ROI to base_footprint [ m ]
const double roi_z_min = -0.5;                                          // minimum hight of ROI [ m ]
const double roi_z_max = 5.0;                                           // maximum hight of ROI [ m ]

// Parameter RANSAC general
const int max_iterations = 1000;                                        // maximum attempts of RANSAC-algorithm
const Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);            // Axis perpendicular to which a plane is to be searched for 
const double distance_threshold = 0.3;                                  // maximum deviation from the plane to be considered as inlier in [ m ]    
const double prob = 0.99;                                               // probability of choosing at least one sample free from outliers 0.99 = 99%

// Parameter  Velodyne front
const double vf_front_length = 32.5;                                    // length of front zone of the front velodyne [ m ]
const double vf_rear_length = 16.5;                                     // length of rear zone of the front velodyne [ m ]
const double vf_deviation_mid_point = 1.5;                              // deviation of the mid point of the two zones from base footprint  [ m ]
const double vf_max_angle_front = perc2rad(3);                          // angular deviation to z axis [ rad ]
const double vf_z_min_ground_front = -0.5;                              // minimum hight for RANSAC [ m ]
const double vf_z_max_ground_front = 0.5;                               // maximum hight for RANSAC [ m ]
const double vf_max_angle_rear = perc2rad(3);                           // angular deviation to z axis [ m ]
const double vf_z_min_ground_rear = -0.5;                               // minimum hight for RANSAC [ m ]
const double vf_z_max_ground_rear = 0.5;                                // maximum hight for RANSAC [ m ]

// Parameter  Velodyne rear
const double vr_front_length = 31.5;                                    // length of front zone of the rear velodyne [ m ]
const double vr_rear_length = 18.5;                                     // length of rear zone of the rear velodyne [ m ]
const double vr_deviation_mid_point = -1.5;                             // deviation of the mid point of the two zones from base footprint [ m ]
const double vr_max_angle_front = perc2rad(3);                          // angular deviation to z axis [ rad ]
const double vr_z_min_ground_front = -0.5;                              // minimum hight for RANSAC [ m ]
const double vr_z_max_ground_front = 0.5;                               // maximum hight for RANSAC [ m ]
const double vr_max_angle_rear = perc2rad(3);                           // angular deviation to z axis [ rad ]
const double vr_z_min_ground_rear = -0.5;                               // minimum hight for RANSAC [ m ]
const double vr_z_max_ground_rear = 0.5;                                // maximum hight for RANSAC [ m ]

// Parameter  Velodyne top
const double vt_front_length = 40.0;                                    // length of front zone of the top velodyne [ m ]
const double vt_deviation_start_point = 20.0;                           // deviation of the rear point of zone of the top velodyne [ m ]
const double vt_max_angle_front = perc2rad(3);                          // angular deviation to z axis [ rad ]
const double vt_z_min_ground_front = -1.0;                              // minimum hight for RANSAC [ m ]
const double vt_z_max_ground_front = 1.0;                               // maximum hight for RANSAC [ m ]

// Parameter  livox front
const double l_front_length = 46.0;                                     // front zone of the livox[ m ]
const double l_rear_length = 10.0;                                      // rear zone of the livox [ m ]
const double l_max_angle_front = perc2rad(3);                           // angular deviation to z axis [ rad ]
const double l_z_min_ground_front = -0.5;                               // minimum hight for RANSAC [ m ]
const double l_z_max_ground_front = 0.5;                                // maximum hight for RANSAC [ m ]
const double l_max_angle_rear = perc2rad(3);                            // angular deviation to z axis [ rad ]
const double l_z_min_ground_rear = -0.5;                                // minimum hight for RANSAC [ m ]
const double l_z_max_ground_rear = 0.5;                                 // maximum hight for RANSAC [ m ]

// Voxelgrid Filter
const int points_per_voxel = 2;                                         // minimum points per voxel
const double voxel_size = 0.1;                                          // edge length of each voxel [ m ]

/**
 * @brief get radians from degree
 * 
 * @param degree angle in radian
 * @return const double angle in radian
 */
const double degree2rad(double degree){
    return degree*M_PI/180;
}

/**
 * @brief percent to radian
 * 
 * @param percent slope in percent
 * @return const double radian
 */
const double perc2rad(double percent){
    return atan(percent/100);
}

/**
 * @brief Get the z threshold object
 * 
 * @param length 
 * @param max_angle 
 * @return const double 
 */
const double get_z_threshold(double length, double max_angle){
    double percent;
    double z_threshold;
    percent = tan(max_angle); // Bsp.: 0.05
    z_threshold = percent * length / 2;
    //ROS_INFO(z_threshold);
}