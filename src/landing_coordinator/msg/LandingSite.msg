# A static candidate site in the coordinator's declared common frame.
# verified means independently checked against the simulation world; it is
# never inferred merely from an ArUco observation.
string site_id
geometry_msgs/Point position
float64 clearance_radius_m
bool verified
bool occupied
