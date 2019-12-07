import numpy as np
# from classDefs import *

def wrap2pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def wrappath(a, path):
	out = a - (len(path.x_d) )* np.floor(a/(len(path.x_d) ))
	out1=out.astype(int)
	# print(out)
	return out1

def simulator( snowbot, controller):

	r = snowbot.r; x0 = snowbot.x0_actual; c =snowbot.c; w_l = snowbot.w_l; w_r = snowbot.w_r; 
	theta = snowbot.state.theta; dt = controller.dt_ctrl

	# x = snowbot.state.x + dt * ( (w_l+w_r)*r*np.cos(theta)/2.0 + (w_r - w_l)*r*x0*np.sin(theta) )

	x_out = snowbot.state.x + dt*( (w_l+w_r)*np.cos(theta)*r/(2.0) + (w_r - w_l)*np.sin(theta)*r*x0/(2.0*c) )# + np.random.normal(0, 0.1, 1)

	y_out = snowbot.state.y + dt*( (w_l+w_r)*np.sin(theta)*r/(2.0) - (w_r - w_l)*np.cos(theta)*r*x0/(2.0*c) ) #+ np.random.normal(0, 0.1, 1)

	theta_out = theta + dt*(w_r - w_l)*r/(2.0*c) #+ np.random.normal(0, 0.01, 1)

	return x_out, y_out, theta_out

# vectorized find_nearest point
def closest_node(X, Y, traj):
    point = np.array([X, Y])
    traj1 = np.asarray(traj)
    dist = point.T - traj1
    dist_2 = np.sum(dist ** 2, axis=1)
    minIndex = np.argmin(dist_2)
    return np.sqrt(dist_2[minIndex]), minIndex