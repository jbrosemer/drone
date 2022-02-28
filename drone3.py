import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math
import matplotlib.patches as mpatches
import random
dt = 1
mu = 0.0
sigmasquared = 1.7
def accelerate(steps):
    viminus1 = 1
    vi = 0
    ximinus1 = 5
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

def newaccelerate(viminus1,ximinus1):
    a = np.random.normal(mu,(sigmasquared))
    vi = a + viminus1
    xi = ximinus1 + viminus1 + a/2
    return a,vi,xi

def repeataccelerate(steps,repeats):
    vels=[]
    poss=[]
    for i in range(repeats):
        i,vi,xi = accelerate(steps)
        vels.append(vi)
        poss.append(xi)
    return vels,poss

sigmagps = 12
R = np.array([[sigmasquared**2/4, math.sqrt(sigmasquared**2)*math.sqrt(sigmasquared**2/4)] , [math.sqrt(sigmasquared**2)*math.sqrt(sigmasquared**2/4), sigmasquared**2]], float)
#R=np.array([[1.7, 0.85], [0.85, 0.425]], float)
A = np.array([[1,1] , [0,1]], float)
B = np.array([[1,6]])
C = np.array([[1 , 0]])
Q = np.array([sigmagps])
#position before is unknown
u = 1
ii = 0
predictedmu = np.array([[0],[0]],float)
zt = 0
E = R
prob = 0.9
a,v,x = newaccelerate(1,5)
poss = []
times = []
predictions = []
while ii < 40:
    measuredmu = np.array(np.dot(A,predictedmu) + B*u)
    #print("measuredmu")
    #print(measuredmu)
    E = np.array(np.dot(np.dot(A,E),np.transpose(A))+R)
    Kalman_Gain = np.array(np.dot(np.dot(E,np.transpose(C)),np.linalg.inv(np.dot(np.dot(C,E),np.transpose(C))+Q)))
    predictedmu = measuredmu + Kalman_Gain*(zt-C*measuredmu)
    predictedmu = np.array([[predictedmu[0][0]],[predictedmu[1][0]]])
    predictedE = np.dot(np.identity(2)-np.dot(Kalman_Gain,C),E)
    #print("predicted mu")
    #print(predictedmu)
    #print(Kalman_Gain)
    #print("Sigma Before")
    #print(E)
    #print("Sigma After")
    #print(predictedE)
    a,v,x = newaccelerate(zt,v)
    rand = random.randint(1,10)
    if rand >= (prob*10)+1:
        x = x + v + a/2
        zt= x + np.random.normal(x,np.sqrt(sigmagps))
    else:
        print("no measurement at time = " + str(ii))    
        zt=predictedmu[1]
    predictions.append(predictedmu[0])
    times.append(ii)
    poss.append(zt)
    ii+=1
plt.clf()
plt.plot(poss, times, 'b')
plt.plot(predictions,times,'g.')
title = ("Robot Position at Time t = " + str(ii-1) + " with chance of losing a measurement p(" + str(prob) + ")")
plt.title(title)
plt.xlabel("Position")
plt.ylabel("Time (Seconds)")
plt.show()