#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
import numpy as np
import random
# index of each variable in the state vector
iX = 0
iV = 1
iA = 2
NUMVARS = 3

class KalmanFilter:
    def __init__(self, initial_x,initial_v,initial_a,initial_t,accel_variance=0.1):
        # mean of state Gaussian distribution
        self.mean = np.zeros(NUMVARS)

        self.mean[iX] = initial_x
        self.mean[iV] = initial_v
        self.mean[iA] = initial_a

        self.cur_time = initial_t

        self._accel_variance = accel_variance

        # covariance of state Gaussian distribution
        self.cov = np.eye(NUMVARS)
        self.cov[iA][iA] = 0.0
        self.cov[iX][iX] = 0.03
        self.cov[iV][iV] = 0.03

    def predict(self, t2,register_change=True):
        dt = t2-self.cur_time
        F = np.eye(NUMVARS)
        F[iX, iV] = dt
        F[iX, iA] = dt*dt*0.5
        F[iV, iA] = dt
        new_mean = F.dot(self.mean)
        new_cov = F.dot(self.cov).dot(F.T)

        G = np.zeros((NUMVARS, 1))
        G[iX] = 0.5*dt*dt
        G[iV] = dt
        G[iA] = 0
        # new_cov += G.dot(G.T) * self._accel_variance#chk this line

        if register_change:
            self.cov = new_cov
            self.mean = new_mean
            self.cur_time += dt
        else:
            return [new_mean,new_cov]

    def update(self, meas_value, meas_variance):
        H = np.zeros((1, NUMVARS))
        H[0, iX] = 1

        z = np.array([meas_value])
        R = np.array([meas_variance])
        y = z - H.dot(self.mean)
        S = H.dot(self.cov).dot(H.T) + R
        K = self.cov.dot(H.T).dot(np.linalg.inv(S))
        new_mean = self.mean + K.dot(y)

        print('self mean, k.y:',self.mean,K.dot(y))

        new_cov = (np.eye(NUMVARS) - K.dot(H)).dot(self.cov)

        self.cov = new_cov
        self.mean = new_mean

    def find_time_required(self, fin_x):

        c1 = self.mean[iX]-fin_x
        b1 = self.mean[iV]
        a1 = 0.5*self.mean[iA]

        if a1>1:
            t1 = (-b1+(b1*b1-4*a1*c1)**0.5)/(2*a1)
            return t1+self.cur_time
        else:            
            return -c1/b1+self.cur_time


ball_coordinates = []
kfx=KalmanFilter(0,0,0,0)
kfy=KalmanFilter(0,0,0,0)
kfz=KalmanFilter(0,0,0,0)

def call_back(msg):
    global ball_coordinates,kfx,kfy,kfz
    ball_coordinates.append(msg)

    if len(ball_coordinates)<=2:
        if len(ball_coordinates)<2:
            return                    
        t1 = (ball_coordinates[0].header.stamp.secs*1000000000+ball_coordinates[0].header.stamp.nsecs)/1000000000.0
        t2 = (ball_coordinates[1].header.stamp.secs*1000000000+ball_coordinates[1].header.stamp.nsecs)/1000000000.0
        vx = (ball_coordinates[1].point.x - ball_coordinates[0].point.x)/(t2-t1)
        vy = (ball_coordinates[1].point.y - ball_coordinates[0].point.y)/(t2-t1)
        vz = ( ball_coordinates[1].point.z - ball_coordinates[0].point.z + (4.9*(t2-t1)*(t2-t1)) )/(t2-t1)

        delta_x = 0.0
        kfx = KalmanFilter(ball_coordinates[0].point.x+delta_x,vx,0.0,t1)
        kfy = KalmanFilter(ball_coordinates[0].point.y+delta_x,vy,0.0,t1)
        kfz = KalmanFilter(ball_coordinates[0].point.z+delta_x,vz,-9.8,t1)

        tpred = kfx.find_time_required(1.5)
        print(t2,kfx.predict(tpred,False)[0][iX],kfy.predict(tpred,False)[0][iX],kfz.predict(tpred,False)[0][iX],tpred,kfx.cur_time)
        print(kfx.mean,kfy.mean,kfz.mean)
        print(' ')
        return 
    
    tnow = (msg.header.stamp.secs*1000000000+msg.header.stamp.nsecs)/1000000000.0
    kfx.predict(tnow)
    kfy.predict(tnow)
    kfz.predict(tnow)

    kfx.update(msg.point.x,0.04)
    kfy.update(msg.point.y,0.04)
    kfz.update(msg.point.z,0.04)

    tpred = kfx.find_time_required(1.5)
    print(tnow,[kfx.predict(tpred,False)[0][iX],kfy.predict(tpred,False)[0][iX],kfz.predict(tpred,False)[0][iX]],tpred)
    print(kfx.mean,kfy.mean,kfz.mean)
    print(' ')

	
def listener():   
    print("trajectory node initiated!!")  
    rospy.init_node('trajectory')
    image_sub = rospy.Subscriber('ball_coordinate',PointStamped,call_back)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


