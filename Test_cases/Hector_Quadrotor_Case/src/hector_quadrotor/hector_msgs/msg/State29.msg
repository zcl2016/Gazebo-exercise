# Vehicle state 'x_hat' output from the estimator or from simulator 

Header header

# Original States
# @warning roll, pitch and yaw have always to be valid, the quaternion is optional
float32[3] position_gps		# latitude (lat. deg), longitude (lon. deg), altitude (m)
float32[3] attitude_angle	# Roll angle (rad), Pitch angle (rad), Yaw angle (rad)
float32[3] velocity		# V_north (m/s), V_east (m/s), V_ground (m/s)
float32[3] angular_velocity	# Body frame rollrate (rad/s), Body frame pitchrate (rad/s), Body frame yawrate (rad/s)
float32[3] acceleration		# Body frame x (m/s^2), Body frame y (m/s^2), Body frame z (m/s^2)
float32    electric_quantity	# Quantity of electric charge
int8       state_word		# Control Status Word
