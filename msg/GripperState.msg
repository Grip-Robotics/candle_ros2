# Per-drive operational state. All arrays share the device_ids order.

std_msgs/Header header

uint32[] device_ids
string[] states # OPEN, CLOSED, HOMING, MOVING, IDLE, or UNKNOWN
