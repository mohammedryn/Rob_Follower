# --- Frame ---
FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2

# --- Following distances (mm) ---
FOLLOW_DIST_MM = 800
STOP_DIST_MM   = 400

# --- Serial ---
SERIAL_PORT = '/dev/ttyACM1'
BAUD        = 115200

# --- Motor speed (PWM units, 0–255) ---
BASE_SPEED = 160
MAX_SPEED  = 220

# --- Steer PID (lateral pixel error → differential speed) ---
KP_STEER         = 0.25
KI_STEER         = 0.001
KD_STEER         = 0.05
STEER_DEADBAND   = 40       # pixels: suppress micro-corrections near center
STEER_OUT_LIMIT  = 100      # max steer contribution (PWM units)
STEER_INTG_LIMIT = 1000
STEER_DERIV_ALPHA = 0.2     # low-pass coefficient for derivative (0=no filter, 1=raw)

# --- Speed PD (distance error → base speed offset; Ki=0 by design) ---
KP_SPEED          = 0.15
KD_SPEED          = 0.02
SPEED_OUT_LIMIT   = 80
SPEED_DERIV_ALPHA = 0.3

# --- Camera ---
LENS_POSITION  = 0.67   # diopters; focus_distance_m = 1/LENS_POSITION ≈ 1.5m
CONF_THRESHOLD = 0.30
