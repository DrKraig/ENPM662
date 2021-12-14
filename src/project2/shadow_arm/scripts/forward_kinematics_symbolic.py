import numpy as np
import sympy as sp

class UR10:
    def __init__(self,q,d):
        self.q = q
        self.d = d
        self.alpha = np.array([90,0,0,90,-90,0])*(sp.pi/180)
        self.a = np.array([0,-0.612,-0.5723,0,0,0])
        self.T_world = sp.eye(4)
    
    def A(self,i):
        a = self.a
        alpha = self.alpha
        d = self.d
        q = self.q.reshape(6,)
        A = sp.Matrix([[sp.cos(q[i]), -sp.sin(q[i])*sp.cos(alpha[i]),
                       sp.sin(q[i])*sp.sin(alpha[i]), a[i]*sp.cos(q[i])],
                      [sp.sin(q[i]), sp.cos(q[i])*sp.cos(alpha[i]),
                       -sp.cos(q[i])*sp.sin(alpha[i]), a[i]*sp.sin(q[i])],
                      [0, sp.sin(alpha[i]), sp.cos(alpha[i]), d[i]],
                      [0, 0, 0, 1]])
        return A   

    def T(self,n):
        T = self.T_world
        for i in range(n):
            T = T * self.A(i)
        return sp.simplify(T)
    

def main():
    # The code is available on
    # https://github.com/DrKraig/ENPM662/tree/devel/src/project2

    d = np.array([0.1273,0,0,0.163941,0.1157,0.0922])    
    q1,q2,q3,q4,q5,q6 = sp.symbols("q1 q2 q3 q4 q5 q6")
    q = np.array([q1,q2,q3,q4,q5,q6])
    q = q.reshape(6,1)
    robot = UR10(q,d)

    # Transformation from world to base of the robot
    robot.T_world = sp.Matrix([[-1,0,0,0],[0,-1,0,0],[0,0,1,0.765],[0,0,0,1]]) 

    print("Transformation matrices from each frame to world frame :")
    for i in range(1,6):
        print("Tranformation matrix of frame",i)
        print(robot.T(i))
        print("---------")
    T_final = robot.T(6)    
    print("Final rotation matrix :")
    print(T_final[0:3,0:3])
    print("---------")
    print("Final translation matrix :")
    print(T_final[0:3,3])

if __name__ == '__main__':
    main()


