# A single map-frame object with calibrated confidence, produced by truth_scorer.
#
# Unlike pointcloud_filters/FusedBuoy, the identity here is owned by the scorer and survives
# LiDAR track death/re-acquisition, and every number is intended to be a probability that has
# been measured against the course answer key rather than a hand-tuned heuristic.

uint32 oid                          # Persistent scorer-owned object id. NOT the LiDAR track id.

# --- Position (map/ENU frame, origin = boat's first GPS fix) ---
float64 east                        # metres
float64 north                       # metres
float64[4] position_cov             # Row-major 2x2 covariance [ee, en, ne, nn] in m^2.
                                    # Anisotropic: tight across bearing, loose along range.

# --- Existence ---
float32 p_exist                     # P(an object physically exists here), 0..1

# --- Class ---
uint32 class_id                     # argmax of the class posterior; 255 = unknown
string class_name                   # human-readable name for class_id
float32 class_prob                  # P(class_id), i.e. the posterior mass at the argmax
float32 class_entropy               # Shannon entropy (nats) of the posterior.
                                    # High = "I don't know" — the honest signal under bad CV.

# Colour marginalised over the whole posterior -- NOT the colour of class_id above. The two
# differ whenever colour mass is spread across several same-colour classes while a colourless
# class takes the argmax: a red cross placard splits its mass between `cross` (colourless) and
# the three red buoy classes, so class_id is `cross` while the colour is confidently "red".
# Colour is what the handbook actually scores, so it gets its own answer rather than being
# derived from the class argmax and silently lost.
string colour                       # "red"/"green"/"yellow"/"black"; "" = none or unknown
float32 colour_prob                 # posterior mass on that colour

# --- Provenance / debugging ---
uint32 observation_count            # Number of measurements fused into this object
uint32 track_id                     # Most recent contributing LiDAR track id (churn debugging)
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
