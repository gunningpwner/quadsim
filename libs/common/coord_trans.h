#ifndef COORDTRANS_H
#define COORDTRANS_H

#include <Eigen/Dense>

const double a = 6378137.0; // semi-major axis
const double f = 1.0 / 298.257223563; // flattening
const double b = a * (1.0 - f); // semi-minor axis
const double e_sq = f * (2.0 - f); // eccentricity squared

double to_radians(double degrees);
Eigen::Vector3f lla_to_ecef(const Eigen::Vector3f& lla);
Eigen::Vector3f lla_to_enu(const Eigen::Vector3f& lla, const Eigen::Vector3f& lla_ref);
Eigen::Matrix3f ecef_to_ned_matrix(double lat_deg, double lon_deg);

#endif // COORDTRANS_H