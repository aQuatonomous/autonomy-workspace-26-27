# Single buoy detection (polar coordinates)
float32 range              # Distance from boat to buoy (meters)
float32 bearing            # Angle from boat's forward axis (radians, -π to +π, CCW per REP 103)
float32 z_mean             # Average height above base_link (meters)
uint32 num_points          # Number of LiDAR points in cluster
float32 lateral_extent     # Bounding box diagonal (meters)
float32 radial_extent      # Depth toward/away from boat (meters)
float32 confidence         # Detection quality score (0.0 to 1.0)
