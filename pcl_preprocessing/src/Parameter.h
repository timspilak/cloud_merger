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

const float perc2rad(float percent);
const float degree2rad(float rad);
const float get_z_threshold(float length, float max_angle);

// outlier
const float radius = 0.15;                                             // radius for points [ m ]
const float min_neighbor = 1;                                          // minimum number of neighbors

// Voxelgrid Filter
const int points_per_voxel = 2;                                        // minimum points per voxel
const float voxel_size = 0.1;                                          // edge length of each voxel [ m ]

// Parameter ROI
const float roi_width  = 10.0;                                         // width of ROI [ m ]
const float roi_length = 75.0;                                         // lenght of ROI [ m ]
const float roi_mid = 15;                                              // distance from rear end of ROI to base_footprint [ m ]
const float roi_z_min = -0.5;                                          // minimum hight of ROI [ m ]
const float roi_z_max = 3.0;                                           // maximum hight of ROI [ m ]

// Parameter RANSAC general
const int max_iterations = 1000;                                       // maximum attempts of RANSAC-algorithm
const Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);           // Axis perpendicular to which a plane is to be searched for 
const float distance_threshold = 0.3;                                  // maximum deviation from the plane to be considered as inlier in [ m ]    
const float prob = 0.99;                                               // probability of choosing at least one sample free from outliers 0.99 = 99%
const float max_angle = perc2rad(5);                                   // angular deviation to z axis [ rad ]

// Parameter  Velodyne front
const float vf_front_length = 28.5;                                    // length of front zone of the front velodyne [ m ]
const float vf_mid_length = 15.0;                                      // length of mid zone of the front velodyne [ m ]
const float vf_mid_length2 = 15.0;                                      // length of mid zone of the front velodyne [ m ]
const float vf_rear_length = 16.5;                                     // length of rear zone of the front velodyne [ m ]
const float vf_deviation_mid_point = 1.5;                              // deviation of the mid point of the two zones from base footprint  [ m ]
const float vf_z_max_ground_front = 2.5;                               // minimum hight for RANSAC [ m ]
const float vf_z_max_ground_mid2 = 2.0;                                 // maximum hight for RANSAC [ m ]
const float vf_z_max_ground_mid = 1.5;                                 // maximum hight for RANSAC [ m ]
const float vf_z_max_ground_rear = 0.5;                                // maximum hight for RANSAC [ m ]

// Parameter  Velodyne rear
const float vr_front_length = 30.0;                                    // length of front zone of the rear velodyne [ m ]
const float vr_mid_length = 31.5;                                      // length of mid zone of the front velodyne [ m ]
const float vr_rear_length = 13.5;                                     // length of rear zone of the rear velodyne [ m ]
const float vr_deviation_mid_point = -1.5;                             // deviation of the mid point of the two zones from base footprint [ m ]
const float vr_z_max_ground_front = 1.0;                               // maximum hight for RANSAC [ m ]
const float vr_z_max_ground_mid = 0.7;                                 // minimum hight for RANSAC [ m ]
const float vr_z_max_ground_rear = 0.5;                                // maximum hight for RANSAC [ m ]

// Parameter  Velodyne top
const float vt_front_length = 40.0;                                    // length of front zone of the top velodyne [ m ]
const float vt_deviation_start_point = 20.0;                           // deviation of the rear point of zone of the top velodyne [ m ]
const float vt_z_max_ground_front = 1.0;                               // maximum hight for RANSAC [ m ]

// Parameter  livox front
const float l_front_length = 26.0;                                     // front zone of the livox[ m ]
const float l_mid_length = 10.0;                                       // mid zone of the livox[ m ]
const float l_mid2_length = 10.0;                                      // mid zone of the livox[ m ]
const float l_rear_length = 10.0;                                      // rear zone of the livox [ m ]
const float l_deviation_mid_point = 4.0;                               // deviation of the mid point of the two zones from base footprint [ m ]
const float l_z_max_ground_front = 1.5;                                // maximum hight for RANSAC [ m ]
const float l_z_max_ground_mid2 = 1.2;                                 // maximum hight for RANSAC [ m ]
const float l_z_max_ground_mid = 0.8;                                  // maximum hight for RANSAC [ m ]
const float l_z_max_ground_rear = 0.5;                                 // maximum hight for RANSAC [ m ]


/**
 * @brief get radians from degree
 * 
 * @param degree angle in radian
 * @return const float angle in radian
 */
const float degree2rad(float degree){
    return degree*M_PI/180;
}

/**
 * @brief percent to radian
 * 
 * @param percent slope in percent
 * @return const float radian
 */
const float perc2rad(float percent){
    return atan(percent/100);
}

/**
 * @brief Get the z threshold object
 * 
 * @param length 
 * @param max_angle 
 * @return const float 
 */
const float get_z_threshold(float length, float max_angle){
    float percent;
    float z_threshold;
    percent = tan(max_angle); // Bsp.: 0.05
    z_threshold = percent * length / 2;
    //ROS_INFO(z_threshold);
}