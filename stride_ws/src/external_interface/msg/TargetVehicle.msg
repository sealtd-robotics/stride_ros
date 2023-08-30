
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
float32 lateral_velocity #m/s 
float32 roll #deg
float32 pitch #deg
float32 acceleration_x #m/s^2
float32 acceleration_y #m/s^2
float32 acceleration_z #m/s^2
bool vehicle_brake