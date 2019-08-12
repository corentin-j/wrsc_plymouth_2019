import numpy as np
import matplotlib.pyplot as plt
import filter_lib as fl

buoy = [2,3]

x = np.arange(1,10,0.1)
y = np.sqrt(x)
theta = np.arctan2(y,x)

dx = buoy[0]-x
dy = buoy[1]-y
d = np.sqrt(dx**2+dy**2)
d += (np.random.random(len(x))-0.5)*2
alpha = np.arctan2(dy,dx)
alpha += (np.random.random(len(x))-0.5)*0.2

x_kal = np.array([[0],[0]])
G0 = 10*np.eye(2)
R = 0.6**2*np.eye(2)
kal = fl.Kalman_filter(x_kal,G0,np.eye(2),np.zeros((2,2)),np.eye(2),R,R)
xf_kal, yf_kal = [],[]

x_mf, y_mf = fl.Median_filter(10), fl.Median_filter(10)
x_filtered, y_filtered = [], []

lp = fl.Low_pass_filter(0.1,x_kal)
x_lp,y_lp = [],[]
for i in range(len(x)):
	x_filtered.append(x_mf.median_step((x+d*np.cos(alpha))[i]))
	y_filtered.append(y_mf.median_step((y+d*np.sin(alpha))[i]))
	mes = np.array([[(x+d*np.cos(alpha))[i]],[(y+d*np.sin(alpha))[i]]])
	res = kal.kalman_step(0,mes)
	print(res)
	xf_kal.append(res[0][0,0])
	yf_kal.append(res[0][1,0])

	res = lp.low_pass_next(mes)
	x_lp.append(res[0,0])
	y_lp.append(res[1,0])

#plt.plot(buoy[0],buoy[1],'+')
plt.plot(x+d*np.cos(alpha),y+d*np.sin(alpha),'.')
#plt.plot(x_filtered,y_filtered,'.')
plt.plot(x_lp,y_lp,'.')
plt.plot(x,y,'.')
plt.show()

print(np.std(y))