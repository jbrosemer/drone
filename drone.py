import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math
import matplotlib.patches as mpatches
dt = 1
mu = 0.0
sigmasquared = 1.7
def accelerate(steps):
    viminus1 = 0
    vi = 0
    ximinus1 = 0
    xi = 0
    accel = list(np.random.normal(mu,(sigmasquared),steps))
    vels = [0]
    poss = [0]
    i = 0
    for a in accel:
        i+=1
        vi = a + viminus1
        xi = ximinus1 + viminus1 + a/2
        #print("velocity" + str(i) + " " + str(vi))
        #print("position" + str(i) + " " + str(xi))
        #print("acceleration" + str(i) + " " + str(a))
        vels.append(vi)
        poss.append(xi)
        viminus1 = vi
        ximinus1 = xi
    accel.insert(0, 0)
    return accel[-1],vels[-1],poss[-1]

def repeataccelerate(steps,repeats):
    vels=[]
    poss=[]
    for i in range(repeats):
        i,vi,xi = accelerate(steps)
        vels.append(vi)
        poss.append(xi)
    return vels,poss

def accel_sim(steps,repeats,E):
    vels,poss = repeataccelerate(steps,repeats)
    print("eigy")
    eigs,eigv = np.linalg.eig(E)
    print(eigs)
    print(eigv)
    print(np.cov(E))
    #print(*math.atan(eigv[0][1]/eigv[0][0])*180/math.pi)
    plt.close()
    fig2 = plt.figure()
    ax = fig2.add_subplot(111)
    lim = steps*6
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect('equal', adjustable='datalim')
    ell2 = mpatches.Ellipse((0,0), math.sqrt(eigs[0]*5.991)*2, math.sqrt(eigs[1]*5.991)*2, math.atan(eigv[1][0]/eigv[0][0])*180/math.pi,fill=False, edgecolor="red")
    ax.add_patch(ell2)
    ax.scatter(np.array(poss),np.array(vels), marker=".")
    title = ("Drone velocity vs position t=" + str(N+1))
    plt.title(title)
    plt.xlabel("Position")
    plt.ylabel("Velocity")
    fig2.show()
    plt.show()

def measurement_sim(steps,repeats,E,newE,munew):
    eigs,eigv = np.linalg.eig(E)
    eigs2,eigv2 = np.linalg.eig(newE)
    plt.close()
    fig2 = plt.figure()
    ax = fig2.add_subplot(111)
    lim = steps*6
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect('equal', adjustable='datalim')
    ellipse1 = mpatches.Ellipse((0,0), math.sqrt(eigs[0]*5.991)*2, math.sqrt(eigs[1]*5.991)*2, math.atan(eigv[1][0]/eigv[0][0])*180/math.pi,fill=False, edgecolor="red")
    ellipse2 = mpatches.Ellipse((munew[0],munew[1]), math.sqrt(eigs2[0]*5.991)*2, math.sqrt(eigs2[1]*5.991)*2, math.atan(eigv2[1][0]/eigv2[0][0])*180/math.pi,fill=False, edgecolor="blue")
    ax.add_patch(ellipse1)
    ax.add_patch(ellipse2)
    title = ("Drone velocity vs position t=" + str(N+1))
    plt.title(title)
    plt.xlabel("Position")
    plt.ylabel("Velocity")
    fig2.show()
    plt.show()


R = np.array([[sigmasquared**2/4, math.sqrt(sigmasquared**2)*math.sqrt(sigmasquared**2/4)] , [math.sqrt(sigmasquared**2)*math.sqrt(sigmasquared**2/4), sigmasquared**2]], float)
#R=np.array([[1.7, 0.85], [0.85, 0.425]], float)
A = np.array([[1,1] , [0,1]], float)
t=0
#When t=1
E = R
N=5
while t <= N:
    t+=1
    E = np.dot(np.dot(A,E),np.transpose(A))+R
print(E)

accel_sim(N+1,100,E)

#MEASUREMENT UPDATE
C = np.array([[1 , 0]])
Q = np.array([12])
zt = 18
Kalman_Gain = np.array(np.dot(np.dot(E,np.transpose(C)),np.linalg.inv(np.dot(np.dot(C,E),np.transpose(C))+Q)))
#position before is unknown
measuredmu = np.array([[0],[0]],float)
predictedmu = measuredmu + Kalman_Gain*(zt+C*measuredmu)
predictedmu = np.array([[predictedmu[0][0]],[predictedmu[1][0]]])
predictedE = np.dot(np.identity(2)-np.dot(Kalman_Gain,C),E)
print("predicted mu")
print(predictedmu)
print(Kalman_Gain)
print("Sigma Before")
print(E)
print("Sigma After")
print(predictedE)
measurement_sim(N+1,100,E,predictedE,predictedmu)