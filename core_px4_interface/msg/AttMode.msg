Header Header
int8 Mode           # Attitude mode for the omni-directional vehicle

int8 NORMAL=0       # Normal attitude (tilting towards the direction of force)
int8 MIN_TILT=1     # Minimizes the tilt
int8 ZERO_TILT=2    # Keeps tilt at zero
int8 LOCK_TILT=3    # Locks the tilt to the estimated amount or the ones set in parameters
int8 LOCK_ATT=4     # Locks the roll/pitch to the estimated amount or the ones set in parameters
int8 ESTIMATE=5     # Estimates the optimal tilt and roll/pitch while hovering. Flies like the normal mode
int8 SLOW=6         # Estimates the optimal tilt and roll/pitch while hovering. Very slow attitude change (rate set by a parameter)
