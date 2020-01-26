#!/usr/bin/env python
import numpy as np 
import rospy
from geometry_msgs.msg import Pose
from ros_dmp.srv import *
import matplotlib.pyplot as plt

x_list = []
y_list = []
tmp_x = -0.5
tmp_y = 0 


class DraggableRectangle:
    def __init__(self, rect):
        self.rect = rect
        self.press = None

    def connect(self):
        'connect to all the events we need'
        self.cidpress = self.rect.figure.canvas.mpl_connect(
            'button_press_event', self.on_press)
        self.cidrelease = self.rect.figure.canvas.mpl_connect(
            'button_release_event', self.on_release)
        self.cidmotion = self.rect.figure.canvas.mpl_connect(
            'motion_notify_event', self.on_motion)

    def on_press(self, event):
        'on button press we will see if the mouse is over us and store some data'

        global x_list, y_list, tmp_x, tmp_y


        if event.inaxes != self.rect.axes: return

        contains, attrd = self.rect.contains(event)
        if not contains: return
        print('event contains', self.rect.xy)
        x0, y0 = self.rect.xy
        self.press = x0, y0, event.xdata, event.ydata
        plt.plot([tmp_x, x0], [tmp_y,y0], marker='o', color='red')
        tmp_x = x0
        tmp_y = y0
        x_list.append(x0)
        y_list.append(y0)

    def on_motion(self, event):
        'on motion we will move the rect if the mouse is over us'
        if self.press is None: return
        if event.inaxes != self.rect.axes: return
        x0, y0, xpress, ypress = self.press
        dx = event.xdata - xpress
        dy = event.ydata - ypress
        #print('x0=%f, xpress=%f, event.xdata=%f, dx=%f, x0+dx=%f' %
        #      (x0, xpress, event.xdata, dx, x0+dx))
        self.rect.set_x(x0+dx)
        self.rect.set_y(y0+dy)

        self.rect.figure.canvas.draw()


    def on_release(self, event):
        'on release we reset the press data'
        self.press = None
        self.rect.figure.canvas.draw()

    def disconnect(self):
        'disconnect all the stored connection ids'
        self.rect.figure.canvas.mpl_disconnect(self.cidpress)
        self.rect.figure.canvas.mpl_disconnect(self.cidrelease)
        self.rect.figure.canvas.mpl_disconnect(self.cidmotion)



if __name__ == "__main__":

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(2,2)
    rects = ax.bar(range(1), height=0.1,  width=0.1, align= 'center', color='blue')
    drs = []
    for rect in rects:
        dr = DraggableRectangle(rect)
        dr.connect()
        drs.append(dr)

    plt.show()

    rospy.init_node('learn_dmp_service_test_client')
    req = LearnDMPRequest()

    # Generating a hypothetical trajectory
    x = np.asarray(x_list)
    print(x)
    # x.reshape(9,1)
    y = np.asarray(y_list)
    # y.reshape(9,1)

    # print(y.size())
    z = np.zeros(9)
    # z.reshape(4,1)

    z = np.hstack((z, np.ones(30)))
    z = np.hstack((z, np.ones(10)*0.5))
    o_x = np.linspace(0, 1)
    o_y = np.linspace(0, 1)
    o_z = np.linspace(0, 1)
    o_w = np.linspace(0, 1)

    # Compose service request
    req.header.frame_id = 'base_link'
    req.output_weight_file_name = '/home/dishuuuuu/Desktop/Advanced Robotics/codes/example.yaml'
    req.dmp_name = 'Inverted Parabola 1'                #Inverted Parabola 1, 2, 3, Step Function
    req.header.stamp = rospy.Time.now()
    req.n_bfs = 50                             #number of basis functions
    req.n_dmps = 6
    for i in range(0,x.size):
        pose = Pose()
        pose.position.x = x[i]
        pose.position.y = y[i]
        pose.position.z = 0
        pose.orientation.y = 1.0
        pose.orientation.z = 1.0
        pose.orientation.w = 1.0
        pose.orientation.x = 1.0
        req.poses.append(pose)

    # Call the service
    try:
        service_client = rospy.ServiceProxy('/learn_dynamic_motion_primitive_service', LearnDMP)
        rospy.loginfo(service_client(req))
    except :
        rospy.loginfo("Service call failed")