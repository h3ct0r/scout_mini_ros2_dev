# msg/BatteryMonitorStatus.msg

# Voltage measured by the XY-DJ module (e.g., 12.5V -> 12.5)
float64 voltage

# Time the relay is ON (in seconds) supplying power to the tether. 
# Converts the time from HH:MM:SS format to total seconds, simplifying calculations.
int32 power_on_time_sec

# Relay status: 1 for ON (OP) and 0 for OFF (CL)
int8 is_power_on