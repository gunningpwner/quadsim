#ifndef COORDTRANS_H
#define COORDTRANS_H

#include <Eigen/Dense>
#include "Vector3.h"

const double a = 6378137.0; // semi-major axis
const double f = 1.0 / 298.257223563; // flattening
const double b = a * (1.0 - f); // semi-minor axis
const double e_sq = f * (2.0 - f); // eccentricity squared

double to_radians(double degrees);
Vector3 lla_to_ecef(const Vector3& lla);
Vector3 lla_to_enu(const Vector3& lla, const Vector3& lla_ref);
Eigen::Matrix3f ecef_to_ned_matrix(double lat_deg, double lon_deg);

#endif // COORDTRANS_H