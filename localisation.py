import csv
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, sin, cos, pi, radians, asin

# Constants for column numbers of measurement types within dataset
TPOSX = 1
TPOSY = 2
TORIX = 3
TORIY = 4
TORIZ = 5
TORIW = 6
TLINX = 7
TLINY = 8
TANGZ = 9

# Constants for column numbers of measurement types within dataset
LINX = 1
LINY = 2
ANGZ = 3
COVLINX = 4
COVLINY = 5
COVANGZ = 6
POSX = 1
POSY = 2

# Other constants
STDEV_GPS = 0.7 #metres

def main():

    print('Running localisation.py')
    # Read odometer and GPS readings into arrays
    file = open("odom.csv")
    csvreader = csv.reader(file)
    header = next(csvreader) #time,linear_x,linear_y,angular_z,cov_linear_x,cov_linear_y,cov_angular_z
    meas = []
    for row in csvreader:
        meas.append([int(row[0])/1e9, float(row[LINX]), float(row[LINY]), float(row[ANGZ]), float(row[COVLINX]), float(row[COVLINY]), float(row[COVANGZ]) ]) 
    file.close()
    odoMeas = np.array(meas)

    file = open("gnss.csv")
    csvreader = csv.reader(file)
    header = next(csvreader) #time,position_x,position_y
    meas = []
    for row in csvreader:
        meas.append([int(row[0])/1e9, float(row[POSX]), float(row[POSY])]) 
    file.close()
    gpsMeas = np.array(meas)

    # The following lines are for initial testing and analysis only
    # Get an idea of sampling intervals in the datasets
    # plotSamplingIntervals(odomMeas, gpsMeas)
    # 
    # Check propagaton model by comparing with the real world truth data 
    propCheck(odoMeas)

def loadTruthData():
    # Read truth dataset, convert quaternions into angle about z axis measured positive from the X axis, and store all data in an array

    file = open("ground_truth.csv")
    csvreader = csv.reader(file)
    header = next(csvreader) #time,position_x,position_y,orientation_x,orientation_y,orientation_z,orientation_w,speed_x,speed_y,angular_z
    truth = []
    for row in csvreader:
        q = [float(row[TORIX]), float(row[TORIY]), float(row[TORIZ]), float(row[TORIW])]
        theta = convertQuaternion(q)
        rec = [int(row[0])/1e9, float(row[TPOSX]), float(row[TPOSY]), theta, float(row[TLINX]), float(row[TLINY]), float(row[TANGZ])]
        truth.append(rec) 
    file.close()    

    return np.array(truth)

def convertQuaternion(q):
    #q = [qx, qy, qz, qw]
    #given pure rotation around z, current angle is given by qw and qz by the relation qw = cos (theta/2) and qz = sin (theta/2)
    theta = 2*atan2(q[2],q[3])
    
    return theta

def propCheck(odoMeas):
    # Check propagaton model by comparing with the real world truth data
    truth = loadTruthData()
    initialState = truth[0,0:4] #time, posx, posy, angle w.r.t to positive x

    #Integrate from truth initial state and truth 'odo' readings and compare with truth
    truthOdoMeas = truth[1:,[0,-3,-2,-1]] #time, linear speed and angular speed in car frame
    states = integrateOdoData(initialState, truthOdoMeas)
    plot = plotComparisonWithTruth(states, truth, 'Differences - truth vs integrated state using truth speeds', interpolateflag=False)
    plot.show()

    #Integrate from truth initial state using actual odo measurements and Interpolate truth values to comare with integrated states
    states = integrateOdoData(initialState, odoMeas)
    plot2 = plotComparisonWithTruth(states, truth, 'Differences - truth vs integrated state using actual odometry speeds')
    plot2.show()

    return    

def plotComparisonWithTruth(states, truth, pltTitle, interpolateflag=True):
    # states - integrated array of states with times and position x, y and angles 
    # truth data set
    # INterpolation flag whether to interpolate truth data or not

    times = states[:,0]
    truthTimes = np.array(truth[:,0])

    truthPosX = np.array(truth[:,1])
    truthPosY = np.array(truth[:,2])
    truthAng = np.array(truth[:,3])

    if interpolateflag:
        tmp = np.interp(times, truthTimes, truthPosX)
        truthPosX = tmp
        tmp = np.interp(times, truthTimes, truthPosY)   
        truthPosY = tmp
        tmp = np.interp(times, truthTimes, truthAng)
        truthAng = tmp   

    fig = plt.figure()
    plt.title(pltTitle)
    plt.subplot(3, 1, 1)
    plt.plot(times, states[:,1]-truthPosX)
    plt.ylabel('Position x (m)')
    plt.subplot(3, 1, 2)
    plt.plot(times, states[:,2]-truthPosY)
    plt.ylabel('Position y (m)')
    plt.subplot(3, 1, 3)
    differencedAngles = np.degrees(compareAngles(states[:,3],truthAng))
    plt.plot(times, differencedAngles)
    plt.ylabel('Angular difference (deg)')
    plt.xlabel('Time (secs)')
    
    return plt

def compareAngles(set1,set2):
    # Given 2 angles (radians) compute angular difference with sign, range -180 to +180
    count = len(set1)
    diff = np.ndarray((count,),float)
    for i in range(count):
        diff[i] = pi - abs(abs(set1[i] - set2[i]) - pi);

    return diff

def rotmat(theta):
    # Computes rotation matrix from car frame to map frame via odo-frame, since odo-frame is same as map-frame, I skip this part
    # Usage = vec_map = R*vec_car
    ct = cos(theta)
    st = sin(theta)
    R = np.array([[ct, -st],[st, ct]]) # rotation about z-axis

    return R

def integrateOdoData(initialState, odoMeas):
    # Given initialState, integrate the state using odometer speeds and build an array of states with each time being the time of the odometer reading
    # initial state is a 1-D array - time, posx (m), posy (m), angle (rotation about z, positive from +x, radians)
    # odoMeas - 2-d array (can have multiple odometer reading times)
    # each row - time, lin speed x (m/s), lin speed y(m/s), angular speed rad/s all in car frame
    #
    # Return:
    # states - 2-D array with each row containing time, posx, posy, angle (first row is supplied initial state)
    t0 = initialState[0]
    X0 = initialState[1:] # given initial state posx, posy, theta
    states = [initialState]

    X = np.array([0.0, 0.0, 0.0])
    for m in odoMeas:
        t = m[0]
        dt = t - t0
        thetaDot = m[3] # angular velocity from encoder, rad/sec
        theta = (X[2] + thetaDot*dt)%2*pi #z axis same in odo and car frame

        #Assume in interval dt, car has rotated by angle theta and rotation matrix from car frame to map frame is computed using angle theta/2
        R = rotmat(theta/2) # from car frame to map frame
        carLinVel = np.array([[m[1]],[m[2]]]) #linear velocity(x,y) from encoder in car frame
        mapLinVel = np.dot(R,carLinVel)

        X[0] = X0[0] + mapLinVel[0]*dt
        X[1] = X0[1] + mapLinVel[1]*dt
        X[2] = theta
        states.append(np.array([t, X[0], X[1], X[2]]))
        t0 = t
        X0 = X

    return np.array(states)

def plotSamplingIntervals(odomMeas, gpsMeas):
    odomSamplingIntervals = []
    for i in range(1,len(odomMeas)):
        odomSamplingIntervals.append(odomMeas[i][0]-odomMeas[i-1][0])

    gpsSamplingIntervals = []
    for i in range(1,len(gpsMeas)):
        gpsSamplingIntervals.append(gpsMeas[i][0]-gpsMeas[i-1][0])

    fig = plt.figure()
    plt.subplot(2, 1, 1)
    plt.semilogy(odomSamplingIntervals)
    plt.ylabel('Odometer')
    plt.subplot(2, 1, 2)
    plt.semilogy(gpsSamplingIntervals)
    plt.ylabel('GPS')
    plt.xlabel('Time (secs)')    
    plt.show()

    return

if __name__ == "__main__":
    main()