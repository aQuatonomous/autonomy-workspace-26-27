# Single detection in map frame (east, north in meters)
std_msgs/Header header
float64 east
float64 north
float64 range_m
float64 bearing_global_rad
string source          # "lidar" (raw LiDAR only) or "fused" (LiDAR+CV from fusion)
string class_name      # e.g. red_buoy, green_buoy
uint32 class_id        # 0-22 from CV class_mapping; 255 = unknown
uint32 id              # LiDAR track id
