#ifndef COORDTRANS_H
#define COORDTRANS_H

#include "Vector3.h"

const double a = 6378137.0; // semi-major axis
const double f = 1.0 / 298.257223563; // flattening
const double b = a * (1.0 - f); // semi-minor axis
const double e_sq = f * (2.0 - f); // eccentricity squared

Vector3 lla_to_enu(const Vector3& lla, const Vector3& lla_ref);

#endif // COORDTRANS_H