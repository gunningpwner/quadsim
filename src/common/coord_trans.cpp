#include "coord_trans.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

Vector3 lla_to_ecef(const Vector3& lla) {
    double lat_rad = to_radians(lla.x);
    double lon_rad = to_radians(lla.y);
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);

    double N = a / sqrt(1.0 - e_sq * sin_lat * sin_lat);

    double x = (N + lla.z) * cos_lat * cos(lon_rad);
    double y = (N + lla.z) * cos_lat * sin(lon_rad);
    double z = ((1.0 - e_sq) * N + lla.z) * sin_lat;

    return {(float)x, (float)y, (float)z};
}

Vector3 lla_to_enu(const Vector3& lla, const Vector3& lla_ref) {
    Vector3 ecef = lla_to_ecef(lla);
    Vector3 ecef_ref = lla_to_ecef(lla_ref);

    double dx = ecef.x - ecef_ref.x;
    double dy = ecef.y - ecef_ref.y;
    double dz = ecef.z - ecef_ref.z;

    double lat_ref_rad = to_radians(lla_ref.x);
    double lon_ref_rad = to_radians(lla_ref.y);
    double sin_lat_ref = sin(lat_ref_rad);
    double cos_lat_ref = cos(lat_ref_rad);
    double sin_lon_ref = sin(lon_ref_rad);
    double cos_lon_ref = cos(lon_ref_rad);

    float e = -sin_lon_ref * dx + cos_lon_ref * dy;
    float n = -sin_lat_ref * cos_lon_ref * dx - sin_lat_ref * sin_lon_ref * dy + cos_lat_ref * dz;
    float u = cos_lat_ref * cos_lon_ref * dx + cos_lat_ref * sin_lon_ref * dy + sin_lat_ref * dz;

    return {e, n, u};
}