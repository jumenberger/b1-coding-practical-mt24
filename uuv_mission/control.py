# implementation of a PD controller
def controller(y0, y1, r0, r1):
	# proportional gain
	Kp = 0.18
	# derivative gain
	Kd = 0.65
	# error
	e0 = r0 - y0
	e1 = r1 - y1
	# action
	u = Kp * e0 + Kd * (e0 - e1)
	return u