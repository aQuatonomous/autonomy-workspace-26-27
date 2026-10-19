# LiDAR track with class from CV–LiDAR fusion (same as TrackedBuoy + class_id, class_name)

uint32 id                           # Unique persistent ID from LiDAR tracker
float32 range                       # Distance from sensor (meters)
float32 bearing                     # Angle from sensor (radians)
float32 x                           # Cartesian x position (meters)
float32 y                           # Cartesian y position (meters)
float32 z_mean                      # Average z height (meters)
uint32 class_id                     # From CV: 0-22 per class_mapping; 255 = unknown
string class_name                   # From CV: e.g. "red_buoy", "red_pole_buoy", "green_buoy", "unknown"
float32 confidence                  # Detection confidence (0.0 to 1.0)
uint32 observation_count            # Number of times this buoy has been detected
builtin_interfaces/Time first_seen   # Timestamp when first detected
builtin_interfaces/Time last_seen   # Timestamp of most recent detection
