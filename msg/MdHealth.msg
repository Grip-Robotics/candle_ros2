# Per-drive health of the MD drives managed by the MD node.
# Published periodically on md/health. All arrays share the device_ids order.

std_msgs/Header header

uint32[] device_ids     # Device IDs
bool[] responsive       # False when the drive did not answer the status query
bool[] error            # True when the drive reports an error or its status is unavailable
string[] active_errors  # Human-readable active error/warning flags, empty when healthy
