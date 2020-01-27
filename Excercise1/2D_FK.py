import math 
import matplotlib.pyplot as plt 

l1 = 1
l2 = 0.5 
n_theta = 10 
theta_start = 0
theta_end = math.pi/2
theta1 = []
theta2 = []
x0 = 0 
y0 = 0
x1 = 0 
y1 = 0
x2 = 0 
y2 = 0


def plot_arm(x1,y1,x2,y2):		

	
	plt.cla()
	plt.plot([x0,x1],[y0,y1])
	plt.plot([x1,x2],[y1,y2])
	plt.xlim([0,2])
	plt.ylim([0,2])
	plt.show()
	plt.pause(0.1)


if __name__=="__main__":


	plt.ion()

	theta1 = [0,math.pi/6,math.pi/4]
	theta2 = [0,math.pi/6,math.pi/4]


	figure = plt.figure()
	for t1 in theta1:
		for t2 in theta2: 


			x1 = l1*math.cos(t1)
			y1 = l1*math.sin(t1)

			x2 = x1 + l2*math.cos(t2)
			y2 = y1 + l2*math.sin(t2)

			print("End effector: ", x2,y2)

			plot_arm(x1,y1,x2,y2)



