# PFCamRigMetadata

# Standard header with timestamp of the exact moment the trigger was sent
std_msgs/Header header

# GPS coordinates from /gps/filtered
sensor_msgs/NavSatFix gps
float32 heading

# Current odometry at trigger time
nav_msgs/Odometry odometry

uint32 run_number
uint32 sequence_number
