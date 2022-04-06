// general
#include "ros/ros.h"
#include "std_msgs/String.h"

// math operations
#include <Eigen/Dense>
#include <cmath>

const double perc2rad(double percent);
const double degree2rad(double rad);
const double get_z_threshold(double length, double max_angle);

// outlier
// -----------------------------------------------------------------
const double radius = 0.1;                                              // Radius fuer weitere Punkte in [ m ]
const double min_neighbor = 1;                                          // minimale Anzahl Nachbarn

// Parameter ROI
// -----------------------------------------------------------------
const double roi_width  = 10.0;                                         // Breite der ROI in [ m ]
const double roi_length = 75.0;                                         // laenge der ROI in [ m ]
const double roi_mid = 15;                                              // Abstand des Fzg. Mittelpunkts zum hinteren Ende der ROI in [ m ]
const double roi_z_min = -0.5;                                          // minimale Hoehe in [ m ]
const double roi_z_max = 3.0;                                           // maximale Hoehe im [ m ]

// Parameter RANSAC general
// -----------------------------------------------------------------
const double lane_width =  10.0;                                        // Breite der Punktewolke in [ m ]
const int max_iterations = 1000;                                        // maximale Anzahl Versuche fuer den RANSAC-Algorithmus
const Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);            // Achse zu der rechtwinklig nach einer Ebene gesucht werden soll 
const double distance_threshold = 0.3;                                  // maximale Abweichung zur Ebene um als Inlier zu gelten in [ m ]    
const double prob = 0.99;                                               // probability of choosing at least one sample free from outliers 0.99 = 99%

// Parameter  Velodyne front
// -----------------------------------------------------------------
const double vf_front_length = 15.0;                                    // Laenge der Zone vor dem shuttle in [ m ]
const double vf_mid_length = 10.0;                                      // Laenge der Zone, in der sich das Shuttle befindet in [ m ]
const double vf_rear_length = 10.0;                                     // Laenge der Zone hinter dem Shuttle in [ m ]

const double vf_max_angle_front = perc2rad(3);                          // maximale Winkelabweichung zur Axe in [ Crad ]
const double vf_z_min_ground_front = -0.5;                              // minimale Hoehe fuer RANSAC [ m ]
const double vf_z_max_ground_front = 0.5;                               // maximale Hoehe fuer RANSAC [ m ]

const double vf_max_angle_mid = perc2rad(1);                            // maximale Winkelabweichung zur Axe in [ Crad ]
const double vf_z_min_ground_mid = -0.2;                                // minimale Hoehe fuer RANSAC [ m ]
const double vf_z_max_ground_mid = 0.2;                                 // maximale Hoehe fuer RANSAC [ m ]

const double vf_max_angle_rear = perc2rad(3);                           // maximale Winkelabweichung zur Axe in [ Crad ]
const double vf_z_min_ground_rear = -0.5;                               // minimale Hoehe fuer RANSAC [ m ]
const double vf_z_max_ground_rear = 0.5;                                // maximale Hoehe fuer RANSAC [ m ]

// Parameter  Velodyne rear
// -----------------------------------------------------------------
const double vr_front_length = 10.0;                                    // Laenge der Zone vor dem shuttle in [ m ]
const double vr_mid_length = 10.0;                                      // Laenge der Zone, in der sich das Shuttle befindet in [ m ]
const double vr_rear_length = 15.0;                                     // Laenge der Zone hinter dem Shuttle in [ m ]

const double vr_max_angle_front = perc2rad(3);                          // maximale Winkelabweichung zur Axe in [ Crad ]
const double vr_z_min_ground_front = -0.5;                              // minimale Hoehe fuer RANSAC [ m ]
const double vr_z_max_ground_front = 0.5;                               // maximale Hoehe fuer RANSAC [ m ]

const double vr_max_angle_mid = perc2rad(1);                            // maximale Winkelabweichung zur Axe in [ Crad ]
const double vr_z_min_ground_mid = -0.2;                                // minimale Hoehe fuer RANSAC [ m ]
const double vr_z_max_ground_mid = 0.2;                                 // maximale Hoehe fuer RANSAC [ m ]

const double vr_max_angle_rear = perc2rad(3);                           // maximale Winkelabweichung zur Axe in [ Crad ]
const double vr_z_min_ground_rear = -0.5;                               // minimale Hoehe fuer RANSAC [ m ]
const double vr_z_max_ground_rear = 0.5;                                // maximale Hoehe fuer RANSAC [ m ]

// Parameter  Velodyne top
// -----------------------------------------------------------------
const double vt_front_length = 40.0;                                    // Laenge der Zone vor dem shuttle in [ m ]
const double vt_mid_length = 40.0;                                      // Laenge der Zone, in der sich das Shuttle befindet in [ m ]
const double vt_rear_length = 0.0;                                     // Laenge der Zone hinter dem Shuttle in [ m ]

const double vt_max_angle_front = perc2rad(5);                          // maximale Winkelabweichung zur Axe in [ Crad ]
const double vt_z_min_ground_front = -1.5;                              // minimale Hoehe fuer RANSAC [ m ]
const double vt_z_max_ground_front = 1.5;                               // maximale Hoehe fuer RANSAC [ m ]

const double vt_max_angle_mid = perc2rad(3);                            // maximale Winkelabweichung zur Axe in [ Crad ]
const double vt_z_min_ground_mid = -0.5;                                // minimale Hoehe fuer RANSAC [ m ]
const double vt_z_max_ground_mid = 0.5;                                 // maximale Hoehe fuer RANSAC [ m ]

const double vt_max_angle_rear = perc2rad(5);                           // maximale Winkelabweichung zur Axe in [ Crad ]
const double vt_z_min_ground_rear = -0.5;                               // minimale Hoehe fuer RANSAC [ m ]
const double vt_z_max_ground_rear = 0.5;                                // maximale Hoehe fuer RANSAC [ m ]

// Parameter  livox front
// -----------------------------------------------------------------
const double lf_front_length = 10.0;                                    // Laenge der Zone vor dem shuttle in [ m ]
const double lf_mid_length = 20.0;                                      // Laenge der Zone, in der sich das Shuttle befindet in [ m ]
const double lf_rear_length = 0.0;                                      // Laenge der Zone hinter dem Shuttle in [ m ]

const double lf_max_angle_front = perc2rad(3);                          // maximale Winkelabweichung zur Axe in [ Crad ]
const double lf_z_min_ground_front = -0.5;                              // minimale Hoehe fuer RANSAC [ m ]
const double lf_z_max_ground_front = 0.5;                               // maximale Hoehe fuer RANSAC [ m ]

const double lf_max_angle_mid = perc2rad(3);                            // maximale Winkelabweichung zur Axe in [ Crad ]
const double lf_z_min_ground_mid = -0.5;                                // minimale Hoehe fuer RANSAC [ m ]
const double lf_z_max_ground_mid = 0.5;                                 // maximale Hoehe fuer RANSAC [ m ]

const double lf_max_angle_rear = perc2rad(5);                           // maximale Winkelabweichung zur Axe in [ Crad ]
const double lf_z_min_ground_rear = -0.5;                               // minimale Hoehe fuer RANSAC [ m ]
const double lf_z_max_ground_rear = 0.5;                                // maximale Hoehe fuer RANSAC [ m ]

// Voxelgrid Filter
// -----------------------------------------------------------------
const int points_per_voxel = 2;                                         // minimale anzahl Punkte pro Voxel 
const double voxel_size = 0.1;                                          // Kantenlaenge pro Voxel in [ m ]

// degree to radian
// -----------------------------------------------------------------
const double degree2rad(double rad){
    return rad*M_PI/180;
}

// percent to radian
// -----------------------------------------------------------------
const double perc2rad(double percent){
    return atan(percent/100);
}

// z threshold from lenght and slope
// -----------------------------------------------------------------
const double get_z_threshold(double length, double max_angle){
    double percent;
    double z_threshold;
    percent = tan(max_angle); // Bsp.: 0.05
    z_threshold = percent * length / 2;
    //ROS_INFO(z_threshold);
}