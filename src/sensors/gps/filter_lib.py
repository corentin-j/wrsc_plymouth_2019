import numpy as np

class Kalman_filter():

	def __init__(self,x0,P0,F,B,H,Q,R): # x0, Gx0,A,B,C Galpha, Gbeta
		self.x = x0
		self.P = P0
		self.F = F
		self.B = B
		self.H = H
		self.Q = Q
		self.R = R

	def kalman_predict(self,u):
		self.P = np.matmul(np.dot(self.F,self.P),self.F.T) + self.Q
		self.x = np.matmul(self.F,self.x) + u
		return(self.x,self.P)

	def kalman_correc(self,y):
		S = np.matmul(np.matmul(self.H,self.P),self.H.T) + self.R        
		K = np.matmul(np.matmul(self.P,self.H.T),np.linalg.inv(S))           
		ytilde = y - np.matmul(self.H,self.x)       
		self.P = np.matmul((np.eye(len(self.x))-np.matmul(K,self.H)),self.P)
		self.x = self.x + np.matmul(K,ytilde) 

	def kalman_step(self,u,y, F=None, B=None):
		if type(F) != type(None):
			self.F = F
		if type(B) != type(None):
			self.B = B
		self.kalman_predict(u)
		self.kalman_correc(y)
		return(self.x,self.P)

	def kalman_set_B(self,B):
		self.B = B

	def kalman_set_F(self,F):
		self.F = F

class Extended_kalman_filter():

	def __init__(self,x0,P0,f,F,h,H,Q,R): # x0, Gx0,A,B,C Galpha, Gbeta
		self.x = x0
		self.P = P0
		self.f = f
		self.F = F
		self.h = h
		self.H = H
		self.Q = Q
		self.R = R

	def EKF_predict(self,u):
		Fx = self.F(self.x,u)
		self.x = self.f(self.x,u)
		self.P = np.matmul(np.matmul(Fx,self.P),np.transpose(Fx))+self.Q

	def EKF_update(self,z):
		Hk = self.H(self.x)
		#innovation
		y = z - self.h(self.x)
		S = np.matmul(np.matmul(Hk,self.P),np.transpose(Hk))+self.R
		# Kalman gain
		K = np.matmul(np.matmul(self.P,np.transpose(Hk)),np.linalg.inv(S))
		# Update
		self.x = self.x + np.matmul(K,y)
		I = np.eye(self.P.shape[0])
		self.P = np.matmul((I-np.matmul(K,Hk)), self.P)

	def EKF_step(self,u,z):
		self.EKF_predict(u)
		self.EKF_update(z)
		return (self.x, self.P)

class Median_filter():

	def __init__(self,n=5):
		self.n = n
		self.l = np.zeros(n)

	def median_step(self,measure):
		self.l[:self.n-1] = self.l[1:]
		self.l[self.n-1] = measure
		return np.median(self.l)

class Low_pass_filter():

	def __init__(self,alpha, first_value):
		self.alpha = alpha
		self.previous = first_value

	def low_pass_next(self,new):
		new =  self.alpha*new + (1-self.alpha)*self.previous
		self.previous = new
		return new
