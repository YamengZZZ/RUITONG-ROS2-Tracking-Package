# Tool tracking data message
string tool_name
string transformation_status
bool match_status

# Marker positions (x, y, z for each marker)
float64[] marker_positions

# Acquisition time
string acquisition_time

# Matching error
float64 matching_error

# Transformation matrix (4x4 = 16 elements, row-major)
float64[16] transformation_matrix

# Tip position (x, y, z)
float64[3] tip_position
