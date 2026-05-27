# Currently-detected stockpiles in the world.
# ids[i] is the aruco tag id of the stockpile whose footprint is polygons[i].
# The two arrays are parallel and the same length.
# Only stockpiles detected this tick are included; absence is signaled by
# omission from both arrays, not by an empty Polygon.
std_msgs/Header header
int32[] ids
geometry_msgs/Polygon[] polygons
