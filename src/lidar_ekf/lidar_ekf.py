#!/usr/bin/env python3
import pickle
import numpy as np
import matplotlib.pyplot as plt

with open('data.pickle', 'rb') as k:
    data=pickle.load(k)
time = data['t']
x_init = data['x_init']
y_init = data['y_init']
x = x_init
y = y_init
theta_init = data['th_init']
theta = theta_init
tran_vel = data['v']
tran_vel_var = data['v_var']
rot_vel = data['om']
rot_vel_var = data['om_var']
motion_var = np.diag([tran_vel_var,rot_vel_var,1])
pose_landmark = data['l']
bearing = data['b']
bearing_var = data['b_var']
range_reading = data['r']
range_var = data['r_var']
corr_var = np.diag([range_var,bearing_var])
X_arr = []
Y_arr = []
P = np.zeros(3)
I =np.identity(3)
neha = np.array([[0],[0],[0]])
for i in range(len(time)-1):
	dt = time[i+1]-time[i]
	x = x - (tran_vel[i]/rot_vel[i])*np.sin(theta) + (tran_vel[i]/rot_vel[i])*np.sin(theta+rot_vel[i]*dt)
	X_arr.append(x)
	y = y + (tran_vel[i]/rot_vel[i])*np.cos(theta) - (tran_vel[i]/rot_vel[i])*np.cos(theta+rot_vel[i]*dt)
	Y_arr.append(y)
	theta = theta+rot_vel[i]*dt
	for j in range(len(pose_landmark)):
		px = pose_landmark[j,0]-x
		py = pose_landmark[j,1]-y
		h = np.array([[np.sqrt(px**2+py**2)],[np.arctan(py/px)-theta]], dtype='float')
		H = np.matrix([[px/np.sqrt(px**2+py**2),py/np.sqrt(px**2+py**2),0],[-py/(px**2+py**2),px/(px**2+py**2),-1]], dtype='float')
		A = np.matrix([[1,0,(-(tran_vel[i]/rot_vel[i])*np.cos(theta)+(tran_vel[i]/rot_vel[i])*np.cos(theta+rot_vel[i]*dt))],[0,1,(-(tran_vel[i]/rot_vel[i])*np.sin(theta)+(tran_vel[i]/rot_vel[i])*np.sin(theta+rot_vel[i]*dt))],[0,0,1]], dtype='float')
		P = (A.dot(P).dot(A.T))+motion_var
		Kg = P.dot(H.T).dot(np.linalg.inv((H.dot(P).dot(H.T)+corr_var)))
		Z = np.array([[range_reading[i,j]], [bearing[i,j]]], dtype='float')
		temp = Z-h
		neha = Kg.dot(Z-h)
		x = x + neha[0,0]
		y = y + neha[1,0]
		P = (I-Kg.dot(H)).dot(P)
	print("i is",i)
	print(x)
	print(y)
	X_arr.append(x)
	Y_arr.append(y)
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ax1.plot(X_arr,Y_arr)
plt.show()
print("i m done")
