import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

#d1,d3,d5,d7 = sp.symbols('d1 d3 d5 d7')

d1,d3,d5,d7 = 360.0,420.0,400.0,226.0

class Kuka:
    def __init__(self,q):
        self.q = q.reshape(7,1)*(np.pi/180)
        self.d = np.array([d1,0,d3,0,d5,0,d7])
        self.alpha = np.array([-90,90,90,-90,-90,90,0])*(np.pi/180)
        self.a = np.zeros(7)

    def get_joints(self):
        return self.q

    def set_joints(self,q):
        self.q = q.reshape(7,1)
    
    def A(self,i):
        a = self.a
        alpha = self.alpha
        d = self.d
        q = self.q.reshape(7,)
        A = np.array([[np.cos(q[i]), -np.sin(q[i])*np.cos(alpha[i]),
                       np.sin(q[i])*np.sin(alpha[i]), a[i]*np.cos(q[i])],
                      [np.sin(q[i]), np.cos(q[i])*np.cos(alpha[i]),
                       -np.cos(q[i])*np.sin(alpha[i]), a[i]*np.sin(q[i])],
                      [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
                      [0, 0, 0, 1]])
        return A   

    def o(self,n):
        T = np.identity(4)
        for i in range(n):
            T = T @ self.A(i)
        o = T[0:3,3]
        return o
    
    def z(self,n):
        T = np.identity(4)
        for i in range(n):
            T = T @ self.A(i)
        z = T[0:3,2]
        return z
    
    def J(self):
        j = np.zeros((6,1))
        for i in range(1,3):
            p = np.cross(self.z(i-1),self.o(7)-self.o(i-1)).reshape(3,1)
            p = np.vstack((p,self.z(i-1).reshape(3,1)))
            j = np.hstack((j,p))
        for i in range(4,8):
            p = np.cross(self.z(i-1),self.o(7)-self.o(i-1)).reshape(3,1)
            p = np.vstack((p,self.z(i-1).reshape(3,1)))
            j = np.hstack((j,p))
        return j[:,1:]

    def FVK(self,qdot):
        if len(qdot) >= 7:
            print("Please make sure you enter only 6 joint velocities, ignore joint 3!")
        Xdot = self.J() @ qdot.reshape(6,1)
        return Xdot

    def IVK_circle(self,t):
        omega = 2*np.pi/5
        radius = 500 # cm
        #xdot = 1
        xdot = -radius*omega*np.sin(omega*t)
        ydot = 0
        #zdot = 1 
        zdot = radius*omega*np.cos(omega*t)
        # Assuming that the tool frame doesn't rotate
        Xdot = np.array([xdot,ydot,zdot,0,0,0]).reshape(6,1)
        qdot = np.linalg.inv(self.J()) @ Xdot
        error = Xdot - self.FVK(qdot)
        qdot = np.insert(qdot, 2, 0).reshape(7,1)
        return qdot, error    
        
def main():
    q = np.array([90,10,10,10,90,10,1])
    robot = Kuka(q)
    #print(robot.o(7))
    #print(robot.J()[:3,:])
    end_time = 10
    time_steps = 2000
    dt = end_time/time_steps
    T = np.linspace(0,end_time,time_steps)
    #qdot,error = robot.IVK_circle(1)
    #print(error)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    #ax2 = fig.add_subplot()
    plt.ion()
    for t in T:
        qdot,error = robot.IVK_circle(t)
        q = robot.get_joints()
        #print(error)
        #ax2.plot(error)
        #print(q)
        #print(qdot)
        q += qdot*dt
        #print(q)
        robot.set_joints(q)
        x,y,z = robot.o(7).tolist()
        #print([x,y,z])
        ax.plot(x,y,z,c='k',marker='.')
        #plt.pause(0.01)
    plt.pause(340)


if __name__ == '__main__':
    main()