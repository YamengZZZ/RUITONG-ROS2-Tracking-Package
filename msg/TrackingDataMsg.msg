# Complete tracking data message
std_msgs/Header header

# Gravity vector (x, y, z)
float64[3] gravity_vector

# Number of all detected markers
int32 markers_number

# Number of stray (unmatched) markers
int32 stray_markers_number

# Tool tracking data array
ToolTrackingMsg[] tools
