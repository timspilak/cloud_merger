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
const double radius = 0.3;                                          // Radius fuer weitere Punkte in [ m ]
const double min_neighbor = 1;                                      // minimale Anzahl Nachbarn

// Parameter ROI
const double x_transverse = 20.0;                                   // Laenge des Querbalkens in [ m ]
const double y_transverse = 25.0;                                   // Breite des Querbalkens in [ m ]
const double x_longitudinal = 30.0;                                 // Laenge des Laengsbalkens in [ m ]
const double y_longitudinal = 10.0;                                 // Breite des Laengsbalkens in [ m ]
const double z_min = -0.5;                                          // minimale Hoehe in [ m ]
const double z_max = 3.0;                                           // maximale Hoehe im [ m ]

// Parameter RANSAC general
const double lane_width =  10.0;                                    // Breite der Punktewolke in [ m ]
const double mid_length = 20.0;                                     // Laenge der Zone, in der sich das Shuttle befindet in [ m ]
const double front_length = 50.0;                                   // Laenge der Zone vor dem shuttle in [ m ]
const double rear_length = 10.0;                                    // Laenge der Zone hinter dem Shuttle in [ m ]
const int max_iterations = 100;                                     // maximale Anzahl Versuche fuer den RANSAC-Algorithmus
const Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);        // Achse zu der rechtwinklig nach einer Ebene gesucht werden soll 
const double distance_threshold = 0.3;                              // maximale Abweichung zur Ebene um als Inlier zu gelten in [ m ]    
const double prob = 0.99;                                           // probability of choosing at least one sample free from outliers 0.99 = 99%

// Parameter RANSAC front
const double max_angle_front = perc2rad(4);                         // maximale Winkelabweichung zur Axe in [ Crad ]
const double z_min_ground_front = -1.5;                             // minimale Hoehe fuer RANSAC [ m ]
const double z_max_ground_front = 1.5;                              // maximale Hoehe fuer RANSAC [ m ]

// Parameter RANSAC mid
const double max_angle_mid = perc2rad(2);                           // maximale Winkelabweichung zur Axe in [ Crad ]
const double z_min_ground_mid = -0.5;                               // minimale Hoehe fuer RANSAC [ m ]
const double z_max_ground_mid = 0.5;                                // maximale Hoehe fuer RANSAC [ m ]


// Parameter RANSAC rear
const double max_angle_rear = perc2rad(4);                          // maximale Winkelabweichung zur Axe in [ Crad ]
const double z_min_ground_rear = -0.5;                              // minimale Hoehe fuer RANSAC [ m ]
const double z_max_ground_rear = 0.5;                               // maximale Hoehe fuer RANSAC [ m ]


// Voxelgrid Filter
const int points_per_voxel = 5;                                     // minimale anzahl Punkte pro Voxel 
const double voxel_size = 0.2;                                      // Kantenlaenge pro Voxel in [ m ]

// Functions

const double degree2rad(double rad){
    return rad*M_PI/180;
}

const double perc2rad(double percent){
    return atan(percent/100);
}

const double get_z_threshold(double length, double max_angle){
    double percent;
    double z_threshold;
    percent = tan(max_angle); // Bsp.: 0.05
    z_threshold = percent * length / 2;
    //ROS_INFO(z_threshold);
}