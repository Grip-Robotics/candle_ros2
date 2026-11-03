# Per-drive health of the MD drives managed by the MD node.
# Published periodically on md/health. All arrays share the device_ids order.

std_msgs/Header header

uint32[] device_ids     # Device IDs
bool[] responsive       # False when the drive did not answer the status query
bool[] error            # True when the drive reports an error or its status is unavailable
string[] active_errors  # Human-readable active error/warning flags, empty when healthy
float32[] motor_temperature_c          # NaN if the temperature read failed; 0 may mean no thermistor
float32[] mosfet_temperature_c         # Controller power-stage temperature; NaN on read failure
float32[] motor_shutdown_temperature_c # Configured motor cutoff, read-only telemetry; NaN on read failure
