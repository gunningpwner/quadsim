#include "WMMData.h"

using namespace wmm;
void calcIncAndDec(const float lat_deg, const float lon_deg, float& inc_rad, float& dec_rad){
    // You give me lat and lon, i give you dec and inc ok??
    // idk what to do in the event that the input is outside the bounds of the table
    if (lat_deg<MIN_LAT || lat_deg>MAX_LAT || lon_deg<MIN_LON || lon_deg>MAX_LON){
        return;
    }

    int lat_idx = (lat_deg-MIN_LAT)/LAT_STEP_SIZE;
    int lon_idx = (lon_deg-MIN_LON)/LON_STEP_SIZE;
    
    // grab values and perform bilinear interpolation
    float lat_norm = fmod(lat_deg - MIN_LAT, LAT_STEP_SIZE) / LAT_STEP_SIZE;
    float lon_norm = fmod(lon_deg - MIN_LON, LON_STEP_SIZE) / LON_STEP_SIZE;

    // Data is stored as {inclination, declination}
    // Index for (lat_idx, lon_idx)
    int idx00 = (lat_idx * LON_STEPS + lon_idx) * 2;
    // Index for (lat_idx, lon_idx + 1)
    int idx01 = (lat_idx * LON_STEPS + (lon_idx + 1)) * 2;
    // Index for (lat_idx + 1, lon_idx)
    int idx10 = ((lat_idx + 1) * LON_STEPS + lon_idx) * 2;
    // Index for (lat_idx + 1, lon_idx + 1)
    int idx11 = ((lat_idx + 1) * LON_STEPS + (lon_idx + 1)) * 2;

    float inc00 = DATA[idx00];
    float dec00 = DATA[idx00 + 1];

    float inc01 = DATA[idx01];
    float dec01 = DATA[idx01 + 1];

    float inc10 = DATA[idx10];
    float dec10 = DATA[idx10 + 1];

    float inc11 = DATA[idx11];
    float dec11 = DATA[idx11 + 1];

    // Interpolate inclination
    float inc_interp_0 = inc00 * (1 - lon_norm) + inc01 * lon_norm;
    float inc_interp_1 = inc10 * (1 - lon_norm) + inc11 * lon_norm;
    inc_rad = inc_interp_0 * (1 - lat_norm) + inc_interp_1 * lat_norm;

    // Interpolate declination
    float dec_interp_0 = dec00 * (1 - lon_norm) + dec01 * lon_norm;
    float dec_interp_1 = dec10 * (1 - lon_norm) + dec11 * lon_norm;
    dec_rad = dec_interp_0 * (1 - lat_norm) + dec_interp_1 * lat_norm;
    return;

}
