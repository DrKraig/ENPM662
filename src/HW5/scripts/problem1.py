import numpy as np
import sympy as sp

d1,d3,d5,d7 = sp.symbols('d1 d3 d5 d7')

class Kuka:
    def __init__(self,q = np.zeros(7)):
        self.q = q*(np.pi/180)
        self.d = np.array([d1,0,d3,0,d5,0,d7])
        self.alpha = np.array([-90,90,90,-90,-90,90,0])*(np.pi/180)
        self.a = np.zeros(7)

    def get_joints(self):
        return self.q

    def set_joints(self,q):
        self.q = q
    
    def A(self,i):
        a = self.a
        alpha = self.alpha
        d = self.d
        q = self.q
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
        radius = 50 # cm
        xdot = radius*np.cos(omega*t)
        ydot = 0
        zdot = radius*np.sin(omega*t)
        # Assuming that the tool frame doesn't rotate
        Xdot = np.array([xdot,ydot,zdot,0,0,0])
        qdot = np.linalg.inv(self.J()) @ Xdot.reshape(6,1)
        return qdot    
        
def main():
    q = np.array([0,0,0,0,0,0,0])
    robot = Kuka(q)
    print(robot.J())
    end_time = 5
    time_steps = 1000
    dt = end_time/time_steps
    T = np.linspace(0,end_time,time_steps)
    for t in T:
        qdot = robot.IVK_circle(t)
        Xdot = robot.FVK(qdot)
        q = robot.get_joints()
        q += qdot*dt
        robot.set_joints(q)


if __name__ == '__main__':
    main()