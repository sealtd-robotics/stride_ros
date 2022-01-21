
uint8 GPS_FIX=0
uint8 DGPS_RTK_FLOAT=1
uint8 DGPS_RTK_INTERGER=2

float32 heading # deg
float64 longitude # deg
float64 latitude # deg
float32 velocity # m/s longitudinal 
bool gps_ready
uint8 gps_correction_type