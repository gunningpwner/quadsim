#include "utils.h"
#include <cmath>

// WGS84 ellipsoid constants
const double a = 6378137.0; // semi-major axis
const double f = 1.0 / 298.257223563; // flattening
const double b = a * (1.0 - f); // semi-minor axis
const double e_sq = f * (2.0 - f); // eccentricity squared

// Converts degrees to radians
double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

Vector3 lla_to_enu(const Vector3& lla, const Vector3& lla_ref) {
    double lat_rad = to_radians(lla.x);
    double lon_rad = to_radians(lla.y);
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);

    double N = a / sqrt(1.0 - e_sq * sin_lat * sin_lat);

    double x = (N + lla.z) * cos_lat * cos(lon_rad);
    double y = (N + lla.z) * cos_lat * sin(lon_rad);
    double z = ((b * b) / (a * a) * N + lla.z) * sin_lat;

    double lat_ref_rad = to_radians(lla_ref.x);
    double lon_ref_rad = to_radians(lla_ref.y);
    double sin_lat_ref = sin(lat_ref_rad);
    double cos_lat_ref = cos(lat_ref_rad);

    double N_ref = a / sqrt(1.0 - e_sq * sin_lat_ref * sin_lat_ref);

    double x_ref = (N_ref + lla_ref.z) * cos_lat_ref * cos(lon_ref_rad);
    double y_ref = (N_ref + lla_ref.z) * cos_lat_ref * sin(lon_ref_rad);
    double z_ref = ((b * b) / (a * a) * N_ref + lla_ref.z) * sin_lat_ref;

    double dx = x - x_ref;
    double dy = y - y_ref;
    double dz = z - z_ref;

    double sin_lon_ref = sin(lon_ref_rad);
    double cos_lon_ref = cos(lon_ref_rad);

    float e = -sin_lon_ref * dx + cos_lon_ref * dy;
    float n = -sin_lat_ref * cos_lon_ref * dx - sin_lat_ref * sin_lon_ref * dy + cos_lat_ref * dz;
    float u = cos_lat_ref * cos_lon_ref * dx + cos_lat_ref * sin_lon_ref * dy + sin_lat_ref * dz;

    return {e, n, u};
}
