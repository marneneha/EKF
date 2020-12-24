#!/usr/bin/env python3
import pickle
import numpy as np
import matplotlib.pyplot as plt

with open('data.pickle', 'rb') as k:
    data=pickle.load(k)
#print(data)
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
motion_var = np.diag(tran_vel_var,rot_vel_var,0)
pose_landmark = data['l']
bearing = data['b']
bearing_var = data['b_var']
range_reading = data['r']
range_var = data['r_var']
corr_var = np.diag(range_var,bearing_var)
X_arr = []
Y_arr = []
P = np.zeros(3)
I =np.identity(3)
for i in range(len(time)-1):
	dt = time[i+1]-time[i]
	x = x - (tran_vel[i]/rot_vel[i])*np.sin(theta) + (tran_vel[i]/rot_vel[i])*np.sin(theta+rot_vel[i]*dt)
	#X_arr.append(x)
	y = y + (tran_vel[i]/rot_vel[i])*np.cos(theta) - (tran_vel[i]/rot_vel[i])*np.cos(theta+rot_vel[i]*dt)
	#Y_arr.append(y)
	theta = theta+rot_vel[i]*dt
	for j in range(len(pose_landmark)):
		px = pose_landmark[j,0]-x
		py = pose_landmark[j,1]-y
		h = [[np.sqrt(px**2+py**2)],[np.arctan(py/px)-theta]]
		H = [[px/np.sqrt(px**2+py**2),py/np.sqrt(px**2+py**2),0],[-py/(px**2+py**2),px/(px**2+py**2),-1]]
		A = [[1,0,(-(tran_vel[i]/rot_vel[i])*np.cos(theta)+(tran_vel[i]/rot_vel[i])*np.cos(theta+rot_vel[i]*dt))],[0,1,(-(tran_vel[i]/rot_vel[i])*np.sin(theta)+(tran_vel[i]/rot_vel[i])*np.sin(theta+rot_vel[i]*dt))],[0,0,1]]
		P = A.dot(P).(A.T)+motion_var
		Kg = P.dot(H.T)/(H.dot(P).dot(H.T)+corr_var)
		Z = [[range_reading[i], bearing[i]]
		x = x + Kg*(Z[0]-h[0])
		y = y + Kg*(Z[1]-h[1])
		P = (I-Kg*H)*P
	X_arr.append(x)
	Y_arr.append(y)
print("plot for motion model only")
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ax1.plot(X_arr,Y_arr)
plt.show()
