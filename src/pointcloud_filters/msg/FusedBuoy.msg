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

# --- appended for truth_scorer; existing subscribers are unaffected -------------------------
float32 class_confidence            # CV class score (0.0-1.0), 0.0 if no CV match.
                                    # NOT the same as `confidence` above, which is the LiDAR
                                    # point-count heuristic min(1, n_points/15) and says nothing
                                    # about what the object is.
builtin_interfaces/Time class_stamp  # Fusion-node clock time at which the CV frame behind
                                    # class_id was received. Zero if no CV match. Fusion
                                    # republishes every new LiDAR scan with whatever CV it last
                                    # had, so without this a consumer cannot tell a fresh class
                                    # from a one-second-old one.
