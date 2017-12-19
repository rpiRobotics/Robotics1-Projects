#!/usr/bin/env python

"""
Author: Zac Ravichandran
Implementation of gradient descent and Newton's method 
for solving v,w for a given waypoint
"""
import numpy as np

def x_t(w, t, v, phi):
	"""
	Get x position after driving for t seconds at w, v
	with initial heading of phi
	"""
	return -v/w * np.cos(w*t + phi) + v/w * np.cos(phi)

def y_t(w, t, v, phi):
	"""
	Get y position after driving for t seconds at w, v
	with initial heading of phi
	"""
	return v/w * np.sin(w*t + phi) - v/w * np.sin(phi)

def dx_w(w, t, v, phi):
	"""
	Partial of x position w.r.t. w
	"""
	return (np.cos(w*t + phi)*v)/np.power(w,2) + \
		(np.sin(w*t + phi)*v*t)/w - (np.cos(phi)*v)/np.power(w,2)

def dy_w(w, t, v, phi):
	"""
	Partial of y position w.r.t. w
	"""
	return -np.sin(w*t + phi)*v/(np.power(w,2)) + \
		 (np.cos(w*t + phi) *v*t)/w+ v*np.sin(phi)/np.power(w,2)

def dx_t(w,t,v,phi):
	"""
	Partial of x position w.r.t t
	"""
	return v * np.sin(w*t + phi)

def dy_t(w, t, v, phi):
	"""
	Partial of y position w.r.t t
	"""
	return v * np.cos(w*t + phi)


def get_wt_from_gradient_descent(xd, yd, v, phi, steps = 100, w0=0.1, t0 = 0.1, k=1):
	"""
	Uses gradient descent to find a w and t that will drive the car to the desired
	x and y location
	"""
	wt = np.reshape(np.array([0.1, 0.1]), (2,1))
	pd = np.reshape(np.array([yd, xd]), (2,1))

	for i in range(steps):
		e = np.reshape(np.array([y_t(wt[0], wt[1], v, phi), x_t(wt[0], wt[1], v, phi)]), (2,1)) - pd 
		grad_e_wt = np.squeeze(np.array([[dy_w(wt[0], wt[1], v, phi), dy_t(wt[0], wt[1], v, phi)],\
								[dx_w(wt[0], wt[1], v, phi), dx_t(wt[0], wt[1], v, phi)]]), axis=2)
		wt = wt - k*np.matmul(np.transpose(grad_e_wt), e)

	return wt[0], wt[1]

def get_wt_from_newtonian_descent(xd, yd, v, phi, steps = 100, w0=0.1, t0 = 0.1, k=1):
	"""
	Uses newtonian descent to find a w and t that will drive the car to the desired
	x and y location
	"""
	wt = np.reshape(np.array([0.1, 0.1]), (2,1))
	pd = np.reshape(np.array([yd, xd]), (2,1))

	for i in range(steps):
		e = np.reshape(np.array([y_t(wt[0], wt[1], v, phi), x_t(wt[0], wt[1], v, phi)]), (2,1)) - pd 
		grad_e_wt = np.squeeze(np.array([[dy_w(wt[0], wt[1], v, phi), dy_t(wt[0], wt[1], v, phi)],\
								[dx_w(wt[0], wt[1], v, phi), dx_t(wt[0], wt[1], v, phi)]]), axis=2)
		wt = wt - k*np.reshape(np.array([np.matmul(np.linalg.pinv(np.reshape(grad_e_wt[:,0], (2,1))), e),\
							 np.matmul(np.linalg.pinv(np.reshape(grad_e_wt[:,1], (2,1))), e)]), (2,1))


	return wt[0], wt[1]

def get_final_x_y(w,t,v,phi):
	return x_t(w,t,v,phi), y_t(w,t,v,phi)

if __name__ == "__main__":
	w, t = get_wt_from_newtonian_descent(-0.2, 0.5, 0.5, 0, steps=100, k=0.2)
	print(get_final_x_y(w,t,0.5,0))
	print(w)
	print(t)
