import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math
import matplotlib.patches as mpatches
import random
dt = 1
mu = 0.0
sigmasquared = 1.7
sigmagps = 12
def accelerate(steps):
    R = np.array([[sigmasquared**2/4, math.sqrt(sigmasquared**2)*math.sqrt(sigmasquared**2/4)] , [math.sqrt(sigmasquared**2)*math.sqrt(sigmasquared**2/4), sigmasquared**2]], float)
    A = np.array([[1,1] , [0,1]], float)
    t=0
    viminus1 = 0
    vi = 0
    ximinus1 = 0
    xi = 0
    accel = list(np.random.normal(mu,(sigmasquared),steps))
    vels = [0]
    poss = [0]
    i = 0
    E = R
    N=6
    #CHANGE THIS TO CHANGE PROBABILITY OF RAIN
    prob = 0.1
    #position before is unknown
    measuredmu = np.array([[0],[0]],float)
    predictedmu = measuredmu
    C = np.array([[1 , 0]])
    Q = np.array([12])
    Kalman_Gain = np.array(np.dot(np.dot(E,np.transpose(C)),np.linalg.inv(np.dot(np.dot(C,E),np.transpose(C))+Q)))
    for a in accel:
        #randomly find actual position
        E = np.dot(np.dot(A,E),np.transpose(A))+R
        if random.randint(1,10) > (1-prob)*10:
            xi= (ximinus1 + np.random.normal(ximinus1,np.sqrt(sigmagps))) + viminus1 + a/2
            zt = xi
            print('position ' + str(zt))
            predictedmu = measuredmu + Kalman_Gain*(zt+C*measuredmu)
            predictedmu = np.array([[predictedmu[0][0]],[predictedmu[1][0]]])
            predictedE = np.dot(np.identity(2)-np.dot(Kalman_Gain,C),E)
            print("predicted mu")
            print(predictedmu)
            #print(Kalman_Gain)
            print("Sigma Before")
            print(E)
            print("Sigma After")
            print(predictedE)
            #measurement_sim(N+1,100,E,predictedE)
        else: 
            xi = ximinus1 + viminus1 + a/2

            predictedE = E
        vi = a + viminus1
        vels.append(vi)
        poss.append(xi)
        viminus1 = vi
        ximinus1 = xi
    accel.insert(0, 0)
    return accel[-1],vels[-1],poss[-1],E,predictedE,predictedmu

def repeataccelerate(steps,repeats):
    vels=[]
    poss=[]
    for i in range(repeats):
        i,vi,xi,E,predictedE,predictedmu = accelerate(steps)
        vels.append(vi)
        poss.append(xi)
    return vels,poss,E,predictedE,predictedmu

def accel_sim(steps,repeats):
    vels,poss,E,predictedE,predictedmu = repeataccelerate(steps,repeats)

    eigs,eigv = np.linalg.eig(E)
    eigs2,eigv2 = np.linalg.eig(predictedE)
    print(eigs)
    print(eigv)
    print(np.cov(E))
    #print(*math.atan(eigv[0][1]/eigv[0][0])*180/math.pi)
    fig2 = plt.figure()
    ax = fig2.add_subplot(111)
    lim = steps*2
    ax.set_xlim(-1000, 1000)
    ax.set_ylim(-100, 100)
    #ax.set_aspect('equal', adjustable='datalim')
    ellipse1 = mpatches.Ellipse((0,0), math.sqrt(eigs[0]*5.991)*2, math.sqrt(eigs[1]*5.991)*2, math.atan(eigv[1][0]/eigv[0][0])*180/math.pi,fill=False, edgecolor="red")
    ellipse2 = mpatches.Ellipse((predictedmu[1],predictedmu[0]), math.sqrt(eigs2[0]*5.991)*2, math.sqrt(eigs2[1]*5.991)*2, math.atan(eigv2[1][0]/eigv2[0][0])*180/math.pi,fill=False, edgecolor="blue")
    ax.add_patch(ellipse1)
    ax.add_patch(ellipse2)
    title = ("Drone velocity vs position t=" + str(steps+1))
    plt.title(title)
    plt.xlabel("Position")
    plt.ylabel("Velocity")
    plt.show()
iterate = 0
while iterate < 100:
    accel_sim(20,1)
    iterate+=1
