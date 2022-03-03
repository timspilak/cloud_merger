#include <Eigen/Dense>
#include <cmath>
const double rad2degree(double rad);
// outlier
const double radius = 0.3;                                      // Radius fuer weitere Punkte in [ m ]
const double min_neighbor = 1;                                      // minimale Anzahl Nachbarn

// Parameter T-Zone
const double x_transverse = 20.0;                               // Laenge des Querbalkens in [ m ]
const double y_transverse = 25.0;                               // Breite des Querbalkens in [ m ]
const double x_longitudinal = 30.0;                             // Laenge des Laengsbalkens in [ m ]
const double y_longitudinal = 10.0;                             // Breite des Laengsbalkens in [ m ]
const double z_min = -0.5;                                      // minimale Hoehe in [ m ]
const double z_max = 3.0;                                       // maximale Hoehe im [ m ]

// Parameter RANSAC
const double z_min_ground = -0.5;                               // minimale Hoehe fuer RANSAC [ m ]
const double z_max_ground = 0.5;                               // maximale Hoehe fuer RANSAC [ m ]
const int max_iterations = 50;                                  // maximale Anzahl Versuche fuer den RANSAC-Algorithmus
const Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);    // Achse zu der paralles nach einer Ebene gesucht werden soll 
const double max_angle = rad2degree(5);                         // maximale Winkelabweichung zur Axe in [ Crad ]
const double distance_threshold = 0.2;                          // maximale Abweichung zur Ebene um als Inlier zu gelten in [ m ]    

// Voxelgrid Filter
const int points_per_voxel = 5;                                 // minimale anzahl Punkte pro Voxel 
const double voxel_size = 0.5;                                  // Kantenlaenge pro Voxel in [ m ]

// Functions

const double rad2degree(double rad){
    return rad*M_PI/180;
}
