import numpy as np 
import pandas as pd 
import matplotlib.pyplot as plt

# Assume that GPS data is correct 
# Calculate error (distance form 1 point to groundtruth)
def er_cal(x,y,x0,y0,x1,y1):
    A = y1 - y0
    B = x0 - x1
    C = x1 * y0 - x0 * y1

    numerator = abs(A * x + B * y + C)
    denominator = np.sqrt(A**2 + B**2)

    distance = numerator / denominator
    return distance
    
######################################################
#KALMAN FILTER
#
######################################################

#Noise covariance form environtment
Qk = np.eye(4)*0.001 #Qk

#Noise from measurement 
#Assume that error ~ 0.05
Rk = np.eye(4)*0.05 #Rk 
#transistion matrix H
H = np.eye(4)

# F matrix for prediction step 
def get_F (theta, deltaT,wk): 
    F = np.array([[1,0,deltaT*np.cos(theta + deltaT* wk),0],
                 [0,1,deltaT*np.sin(theta + deltaT* wk),0],
                 [0,0,1,0],
                 [0,0,0,1]])
    return F


def kalman_predict(X0,P0,Qk,Fk,ax, w,deltaT) :
    m = deltaT**2 / 2 
    pos = X[3][0]
    B1 = np.array([[m*np.cos(pos)],
                  [m*np.sin(pos)],
                  [deltaT],
                  [0]])
    B2 = np.array([[0],[0],[0],[deltaT]])
    Xp = Fk.dot(X0) + B1.dot(ax) + B2.dot(w) 

    Pp = (Fk.dot(P0)).dot(Fk.T) + Qk
    return Xp, Pp

def kalman_update(Xp,Pp,Hk, Rk,Zk) : 
    K = (Pp.dot( Hk.T)).dot( np.linalg.inv((Hk.dot(Pp)).dot(Hk.T) + Rk))
    X = Xp + K.dot((Zk - Hk.dot( Xp)))
    P = Pp - K.dot((Hk.dot(Pp))) 
    return X, P 

#####################################

# Create new state of robot  
X = np.zeros((4,1))
X[3][0]=np.pi/2 # when start X-axis of robot is 90 degree vs X-axis of the Oxy we define 
P = np.eye(4)*0.2

#read data which made without kalman filter 
no_filter = np.loadtxt("no_filter.csv",delimiter= ",")
# make start point at (0,0)
deltax0 = no_filter[1,0] - 0
deltay0 = no_filter[1,1] - 0

x_nf = no_filter[:,0] - deltax0
y_nf = no_filter[:,1] - deltay0
error_nf = no_filter[:,2]

###############
# Read data from sensors

# Read GPS data
column_names = ['UTIME', 'Fixmode', 'NumberOfSatelines', 'Latitude', 'Longtitude', 'Altitude', 'Track', 'Speed']
gps_data = pd.read_csv('gps.csv', names=column_names)

# Calculate value for GPS 
lat0 = gps_data.iloc[0]['Latitude']
lon0 = gps_data.iloc[0]['Longtitude']
#alt0 = 270
re = 6378135
rp = 6356750

t = re * np.cos(lat0) * re * np.cos(lat0) + rp * np.sin(lat0) * rp * np.sin(lat0)
rns = (re * rp * re * rp) / (t * np.sqrt(t))
rew = re * re / (np.sqrt(t))

# Read IMU data
imu_data = pd.read_csv('ms25.csv', low_memory=False)

imu_data['X_a'] = pd.to_numeric(imu_data['X_a'], errors='coerce')
imu_data['Z_r'] = pd.to_numeric(imu_data['Z_r'], errors='coerce')
# Fusion data from gps and imu 
utime = 0
indexOfImu = 2
x_recorded = []
x_gps = []
y_gps = []
y_recorded = []
length = 1000 # change it to 10000 for better result but it'll take more time
error =[]

# Iterate through GPS data
for i in range(0,length ):
    if (gps_data.iloc[i]['Fixmode'] == 2 or gps_data.iloc[i]['Fixmode'] == 3 ):
        utime = gps_data.iloc[i]['UTIME']
        lat = gps_data.iloc[i]['Latitude']
        lon = gps_data.iloc[i]['Longtitude']
        #get previous x and y 
        x = np.sin(lon - lon0) * rew * np.cos(lat0)
        y = np.sin(lat - lat0) * rns 
        
        #update angle of the robot 
        if i != 0 :
            x_previous = x_gps[-1]
            y_previous = y_gps[-1]

            # check if robot not move so skip calculate angle.
            if x != x_previous:
                angle = np.arctan((y - y_previous) / (x - x_previous))
            else:
                if y != y_previous:
                    angle = np.pi / 2  # 90 degrees if x and x_previous are the same but y is different
                else:
                    continue  # use previous state if both x and y are the same
        else: angle = np.pi/2  
        
        x_gps.append(x)
        y_gps.append(y)
        vx = gps_data.iloc[i]['Speed']
       
        if(pd.isna(vx)) : #check if vx is number or NaN 
            vx = gps_data.iloc[i+1]['Speed']

        #fusion 
        while (utime > imu_data.iloc[indexOfImu]['UTIME']): 
            deltaT = (imu_data.iloc[indexOfImu]['UTIME'] - imu_data.iloc[indexOfImu - 1]['UTIME'])/1000000
            pos = X[3][0]
            a = imu_data.iloc[indexOfImu-1]['X_a']
            w = imu_data.iloc[indexOfImu-1]['Z_r']
            F = get_F(pos,deltaT,w)
            X,P = kalman_predict(X,P,Qk,F,a,w,deltaT)
            #caculate error
            if i != 0:
                error.append(er_cal(X[0][0],X[1][0],x,y,x_previous,y_previous))
            x_recorded.append(X[0][0])
            y_recorded.append(X[1][0])
            indexOfImu+= 1 
      
        #update matrix 
        Z = np.array([[x],[y],[vx],[angle]])
        X,P = kalman_update(X,P,H,Rk,Z)
        x_recorded.append(X[0][0])
        y_recorded.append(X[1][0])
x_nf = x_nf[:indexOfImu] 
y_nf = y_nf[:indexOfImu]
error_nf = error_nf[:indexOfImu]
    
# Plot (x, y)
plt.scatter(x_recorded, y_recorded,1, color='blue',label='filtered')
plt.plot(x_gps, y_gps,color = "green",label='ground truth')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('(X, Y)')

plt.scatter(x_nf,y_nf,1,color="red", label='no filter')
plt.tight_layout()
plt.legend()

# Plot error 
plt.figure()
plt.plot(error, color='blue',label="filter")
plt.plot(error_nf, color='green',label='no filter')
plt.legend()
plt.title('Error')
# Show the plot
plt.show()
