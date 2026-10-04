# Classified motion state and link health, produced by motion_state_monitor.

std_msgs/Header header

# --- motion state ---------------------------------------------------------
uint8 STATE_IDLE            = 0
uint8 STATE_FORWARD         = 1
uint8 STATE_REVERSE         = 2
uint8 STATE_TURN_IN_PLACE   = 3
uint8 STATE_ARC_LEFT        = 4
uint8 STATE_ARC_RIGHT       = 5
uint8 STATE_BRAKING         = 6
uint8 STATE_SAFETY_HOLD     = 7   # watchdog / invalid-input hold
uint8 STATE_EMERGENCY_STOP  = 8   # e-stop latched
uint8 STATE_SIGNAL_LOST     = 9   # no upstream command at all

uint8 state
string state_name

# --- health ---------------------------------------------------------------
uint8 HEALTH_OK    = 0
uint8 HEALTH_WARN  = 1
uint8 HEALTH_ERROR = 2
uint8 HEALTH_STALE = 3

uint8 health
string health_message

# --- kinematics, measured from /odom --------------------------------------
float64 linear_speed              # m/s, signed
float64 angular_speed             # rad/s, signed (CCW positive)
float64 turn_radius               # m; +inf while driving straight
float64 heading                   # rad
float64 distance_travelled        # m, cumulative path length
float64 duration_in_state         # s

# --- link health, measured independently of the guard ---------------------
float64 cmd_input_rate_hz         # /cmd_vel rate as seen by the monitor
float64 safe_cmd_rate_hz          # /cmd_vel_safe rate
float64 time_since_last_cmd       # s since last raw /cmd_vel
float64 tracking_error_linear     # |commanded - measured| m/s
float64 tracking_error_angular    # |commanded - measured| rad/s
bool input_frozen                 # upstream repeating one value for a long time

# --- safety accounting ----------------------------------------------------
uint32 intervention_count
uint16 last_safety_flags          # mirrors SafetyReport/flags
