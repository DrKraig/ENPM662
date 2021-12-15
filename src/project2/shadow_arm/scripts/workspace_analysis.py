import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
class UR10:
    def __init__(self,q,d):
        self.q = q
        self.d = d
        self.alpha = np.array([90,0,0,90,-90,0])*(np.pi/180)
        self.a = np.array([0,-0.612,-0.5723,0,0,0])
        self.T_world = np.identity(4)
    
    def A(self,i):
        a = self.a
        alpha = self.alpha
        d = self.d
        q = self.q.reshape(6,)
        A = np.array([[np.cos(q[i]), -np.sin(q[i])*np.cos(alpha[i]),
                       np.sin(q[i])*np.sin(alpha[i]), a[i]*np.cos(q[i])],
                      [np.sin(q[i]), np.cos(q[i])*np.cos(alpha[i]),
                       -np.cos(q[i])*np.sin(alpha[i]), a[i]*np.sin(q[i])],
                      [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
                      [0, 0, 0, 1]])
        return A   

    def o(self,n):
        T = self.T_world
        for i in range(n):
            T = T @ self.A(i)
        o = T[0:3,3]
        return o

    def z(self,n):
        T = self.T_world
        for i in range(n):
            T = T @ self.A(i)
        z = T[0:3,2]
        return z

    def T(self,n):
        T = self.T_world
        for i in range(n):
            T = T @ self.A(i)
        return T
    
    def J(self):
        j = np.zeros((6,1))
        for i in range(1,7):
            p = np.cross(self.z(i-1),self.o(6)-self.o(i-1)).reshape(3,1)
            p = np.vstack((p,self.z(i-1).reshape(3,1)))
            j = np.hstack((j,p))
        jacobian = j[:,1:]
        if abs(np.linalg.det(jacobian)) <= 1e-4:
            return 0
        else:
            return 1

def main():
    # The code is available on
    # https://github.com/DrKraig/ENPM662/tree/devel/src/project2
    print("Workspace Analysis")
    d = np.array([0.1273,0,0,0.163941,0.1157,0.0922])    
    q = np.zeros(6)
    # q = np.array([0.09523985696467108, -0.7183862276793844, 0.48845086571663465, 0.22925462512416583, 1.6656604751690187, -1.571821596664277])
    q = q.reshape(6,1)
    robot = UR10(q,d)
    robot.T_world = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0.765],[0,0,0,1]])

    fig = plt.figure(figsize=(8, 4))
    gs = gridspec.GridSpec(nrows=1, ncols=2)
    ax1 = fig.add_subplot(gs[0, 0], projection='3d')
    ax2 = fig.add_subplot(gs[0, 1])
    plt.ion()
    ax1.set_box_aspect((1, 1, 1))
    ax2.axis("equal")
    #plt.axis('auto')
    min_r = 2
    for i in range(12000):
        q = np.random.rand(6,1)*2*np.pi
        robot.q = q
        o = robot.o(6)
        r = np.linalg.norm([o[0],o[1]])
        if r < min_r:
            min_r = r
        ax1.plot(o[0],o[1],o[2],marker="o",markersize=1,color='b')
        ax2.plot(o[0],o[1],marker="o",markersize=1,color='b')
    print("Minimum distance of EE from origin in X-Y plane : ",min_r)
    ax1.set_xlabel("X axis")
    ax1.set_ylabel("Y axis")
    ax1.set_zlabel("Z axis")
    ax1.set_title("Workspace of UR10 Arm 3D")
    ax2.set_xlabel("X axis")
    ax2.set_ylabel("Y axis")
    ax2.set_title("Workspace of UR10 Arm X-Y Plane")
    plt.pause(340)




if __name__ == '__main__':
    main()


