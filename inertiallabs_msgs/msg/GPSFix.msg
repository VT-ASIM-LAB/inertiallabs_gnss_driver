uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3

std_msgs/Header header

gps_common/GPSStatus status

float64 latitude
float64 longitude
float64 altitude

float64 track
float64 speed
float64 climb

float64 pitch
float64 roll
float64 dip

float64 time

float64 gdop
float64 pdop
float64 hdop
float64 vdop
float64 tdop

float64 err
float64 err_horz
float64 err_vert

float64 err_track
float64 err_speed
float64 err_climb

float64 err_time

float64 err_pitch
float64 err_roll
float64 err_dip

float64[9] position_covariance
uint8 position_covariance_type


