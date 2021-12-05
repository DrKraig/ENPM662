import numpy as np
import sympy as sp

class Kuka:
    def __init__(self,q,d):
        self.q = q
        self.d = d
        self.alpha = np.array([-90,90,90,-90,-90,90,0])*(np.pi/180)
        self.a = np.zeros(7)
        self.com = np.array([[0,d[0]/2,0,1],
                             [0,0,d[2]/4,1],
                             [0,-d[2]/4,0,1],
                             [0,0,d[4]/4,1],
                             [0,d[4]/4,0,1],
                             [0,0,d[6]/2,1],
                             [0,0,0,1]])

    def get_joints(self):
        return self.q

    def set_joints(self,q):
        self.q = q.reshape(7,1)
    
    def A(self,i):
        a = self.a
        alpha = self.alpha
        d = self.d
        q = self.q.reshape(7,)
        A = np.array([[sp.cos(q[i]), -sp.sin(q[i])*sp.cos(alpha[i]),
                       sp.sin(q[i])*sp.sin(alpha[i]), a[i]*sp.cos(q[i])],
                      [sp.sin(q[i]), sp.cos(q[i])*sp.cos(alpha[i]),
                       -sp.cos(q[i])*sp.sin(alpha[i]), a[i]*sp.sin(q[i])],
                      [0, sp.sin(alpha[i]), sp.cos(alpha[i]), d[i]],
                      [0, 0, 0, 1]])
        return A   

    def o(self,n):
        T = np.identity(4)
        for i in range(n):
            T = T @ self.A(i)
        o = T[0:3,3]
        return o

    def o_com(self,n):
        T = np.identity(4)
        for i in range(n):
            T = T @ self.A(i)
        o = T @ self.com[n-1].reshape(4,1)
        o = (o[0:3]).reshape(3)
        return o
    
    def z(self,n):
        T = np.identity(4)
        for i in range(n):
            T = T @ self.A(i)
        z = T[0:3,2]
        return z

    def T(self,n):
        T = np.identity(4)
        for i in range(n):
            T = T @ self.A(i)
        return T
    
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
        jacobian = j[:,1:]
        return jacobian

    def Jcom(self,n):
        j = np.zeros((6,1))
        for i in range(1,8):
            if i == 3:
                continue
            if i <= n:
                p = np.cross(self.z(i-1),self.o_com(n)-self.o(i-1)).reshape(3,1)
                p = np.vstack((p,self.z(i-1).reshape(3,1)))
                j = np.hstack((j,p))
            else:
                p = np.zeros((6,1))
                j = np.hstack((j,p))
        return j[:,1:]

    def ID(self,F):
        B = self.J().T
        for n in range(1,8):
            jT_new = self.Jcom(n).T
            B = np.hstack((B,jT_new))
        Tau = (B @ F) * 1e-6
        return Tau    

    def FVK(self,qdot):
        if len(qdot) >= 7:
            print("Please make sure you enter only 6 joint velocities, ignore joint 3!")
        Xdot = self.J() @ qdot.reshape(6,1)
        return Xdot

    def IVK_circle(self,t):
        omega = 2*np.pi/200
        radius = 100 # mm
        xdot = -radius*omega*np.sin(omega*t + np.pi/2)
        ydot = 0
        zdot = radius*omega*np.cos(omega*t + np.pi/2)

        # Assuming that the tool frame doesn't rotate
        Xdot = np.array([xdot,ydot,zdot,0,0,0]).reshape(6,1)
        Jinv = np.linalg.inv(self.J())
        qdot = Jinv @ Xdot
        qdot = np.insert(qdot, 2, 0).reshape(7,1)
        return qdot, Jinv

    def show(self,fig,ax):
        ax.clear()
        # Link 1
        j1 = self.o(0)
        j2 = self.o(1)
        j4 = self.o(3)
        j6 = self.o(5)
        ee = self.o(7)

        x = [j1[0],j2[0]]
        y = [j1[1],j2[1]]
        z = [j1[2],j2[2]]
        ax.plot(x,y,z,color='k',linewidth=5)
        x = [j2[0],j4[0]]
        y = [j2[1],j4[1]]
        z = [j2[2],j4[2]]
        ax.plot(x,y,z,color='k',linewidth=5)
        x = [j4[0],j6[0]]
        y = [j4[1],j6[1]]
        z = [j4[2],j6[2]]
        ax.plot(x,y,z,color='k',linewidth=5)
        x = [j6[0],ee[0]]
        y = [j6[1],ee[1]]
        z = [j6[2],ee[2]]
        ax.plot(x,y,z,color='k',linewidth=5)
        A = self.A(0) @ self.A(1) @ self.A(2) @ self.A(3) @ self.A(4) @ self.A(5) @ self.A(6)
        o =  A @ np.array([0,0,0,1]).T
        vx = A[0:3,0:3] @ np.array([100,0,0])
        vy = A[0:3,0:3] @ np.array([0,100,0])
        vz = A[0:3,0:3] @ np.array([0,0,100])
        ax.quiver(o[0],o[1],o[2],vx[0],vx[1],vx[2],color='r')
        ax.quiver(o[0],o[1],o[2],vy[0],vy[1],vy[2],color='g')
        ax.quiver(o[0],o[1],o[2],vz[0],vz[1],vz[2],color='b')
        ax.set_xlim3d(-800,800)
        ax.set_ylim3d(-600,1000)
        ax.set_zlim3d(0,1000)
        fig.canvas.draw()
        fig.canvas.flush_events()

def print_list(a,text):
    print("List of " + text + " :")
    for i in a:
        print(i)
        print("---------")
    print("################################")

def main():
    # The code is available on
    # https://github.com/DrKraig/ENPM662/tree/devel/src/HW5
     
    d1,d3,d5,d7 = 360.0,420.0,399.5,205.5
    d = np.array([d1,0,d3,0,d5,0,d7]) 
    q1,q2,q3,q4,q5,q6,q7 = sp.symbols('q1 q2 q3 q4 q5 q6 q7')   
    q = np.array([q1,q2,q3,q4,q5,q6,q7])
    robot = Kuka(q,d)
    g = 9.81 * 1000 # mm/s^2
    link_mass = 22.3/6
    m = np.full((8),link_mass)

    ###################### Declaring Forces
    F = np.zeros((48,1)) # Kg-mm/s^2 
    # Force on end effector 
    F[1][0] = 0.0
    # Gravity on each link
    F[8][0] = -m[1]*g
    F[14][0] = -m[2]*g
    F[20][0] = -m[3]*g
    F[26][0] = -m[4]*g
    F[32][0] = -m[5]*g
    F[38][0] = -m[6]*g

    joint_labels = ['1','2','4','5','6','7']
    Tau = robot.ID(F)
    for i in range(6):
        expr = sp.nsimplify(Tau[i,0],tolerance=1e-5)
        print("g(q) at Joint",joint_labels[i],"is",expr,"N-m")
        print("##############")

if __name__ == '__main__':
    main()