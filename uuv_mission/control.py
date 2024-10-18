# implementation of a PD controller
def controller(y0, y1, r0, r1):
	# proportional gain
	Kp = 0.15
	# derivative gain
	Kd = 0.6
	# error
	e0 = y0 - r0
	e1 = y1 - r1
	# action
	u = Kp * e0 + Kd * (e0 - e1)
	return u