# Per-cycle record of what the velocity guard did to the incoming command.
#
# Published at the guard's fixed control rate so that the timeline is uniform
# and plottable even while the upstream publisher is silent.

std_msgs/Header header

# --- intervention flags (bitmask, several may be set at once) --------------
uint16 FLAG_NONE                  = 0
uint16 FLAG_NON_FINITE            = 1     # NaN/Inf in input, whole frame dropped
uint16 FLAG_NONHOLONOMIC          = 2     # ly/lz/ax/ay were non-zero, projected to 0
uint16 FLAG_SPIKE_REJECTED        = 4     # isolated jump awaiting confirmation
uint16 FLAG_LINEAR_CLAMPED        = 8     # linear speed magnitude clamped
uint16 FLAG_ANGULAR_CLAMPED       = 16    # angular speed magnitude clamped
uint16 FLAG_LATERAL_ACCEL_LIMITED = 32    # |v*w| centripetal coupling limit hit
uint16 FLAG_ACCEL_LIMITED         = 64    # slew-rate (accel/decel) limit hit
uint16 FLAG_DEADBAND_APPLIED      = 128   # tiny command snapped to zero
uint16 FLAG_WATCHDOG_TIMEOUT      = 256   # input stale, braking
uint16 FLAG_ESTOP                 = 512   # emergency stop engaged
uint16 FLAG_SAFETY_HOLD           = 1024  # too many consecutive invalid frames

uint16 flags
string[] reasons                  # human-readable, parallel to flags

# --- the full input -> target -> output chain, self-documenting ------------
geometry_msgs/Twist input_cmd     # newest raw input this cycle referred to
geometry_msgs/Twist target_cmd    # accepted target after validation
geometry_msgs/Twist output_cmd    # what was actually published

# --- link health as seen by the guard -------------------------------------
float64 input_rate_hz
float64 time_since_last_input
bool watchdog_active
bool estop_active

# --- cumulative counters --------------------------------------------------
uint64 total_received
uint64 total_rejected             # frames dropped entirely
uint64 total_modified             # frames altered in any way
uint64 total_published
