
uint8 GPS_FIX=5
uint8 DGPS_RTK_FLOAT=6
uint8 DGPS_RTK_INTERGER=7

float32 heading # deg
float64 longitude # deg
float64 latitude # deg
float32 velocity # m/s longitudinal 
bool gps_ready
uint8 gps_correction_type
uint8 no_of_satellites