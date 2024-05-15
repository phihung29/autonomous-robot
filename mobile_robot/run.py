#kalman filter
######################################################
#KALMAN FILTER
#
######################################################
import numpy as np
import serial
import mpu6050
import time 
import magnetic as mag 
import pynmea2
from datetime import datetime
import smbus
import math
import serial.serialutil
import control
# Define the serial port and baudrate
gps_port = '/dev/ttyS0'  # Adjust this if your serial port is different
baudrate = 9600  # Adjust this if your GPS module uses a different baudrate
arduino_port = '/dev/ttyUSB0'

# Initialize variables for storing previous GPS data
prev_lat = None
prev_lon = None
prev_time = None

#Noise covariance form environtment
Qk = np.eye(4)*0.001 #Qk
P = np.eye(4)*0.2
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
def gps_to_x_y(lat0,lng0,lat,lng):
        dLat = lat - lat0
        dLng = lng - lng0

        rns = 6343618.3790280195
        rew = 6380879.425381593 

        y = rns*np.sin(dLat)
        x = rew*np.sin(dLng)*np.cos(lat0) 
        
        return x,y 
#khoi tao, di thang 1 doan ngan de xac dinh vi tri va goc. PWM = 50
def read_gps():
    line = gps_ser.readline().decode('utf-8', errors='ignore')

        # Check if the line contains GGA data
    if line.startswith('$GPGGA'):
            # Parse the NMEA sentence
        msg = pynmea2.parse(line)

            # Extract latitude, longitude, and timestamp from the message
        lat = msg.latitude/180*np.pi
        lon = msg.longitude/180*np.pi
        return lat,lon
    
    return None, None
def read_sensor_data():
    # Read the accelerometer values
    accelerometer_data = mpu6050.get_accel_data()
    a  = accelerometer_data ['x']
    # Read the gyroscope values
    gyroscope_data = mpu6050.get_gyro_data()
    w = - (gyroscope_data['z']/180*np.pi) 

    return a,w
def distance_cal(x,y,x_d,y_d): 
    return np.sqrt((x-x_d)**2 + (y-y_d)**2) 
arduino_ser = serial.Serial(arduino_port, 9600,timeout = 1)
gps_ser = serial.Serial(gps_port, baudrate, timeout = 2)
x_p = 0
y_p  = 0

mpu6050 = mpu6050.mpu6050(0x68)
# Define I2C address of the HMC5883L
HMC5883L_ADDR = 0x1E

# Register addresses
MODE_REGISTER = 0x02
DATA_REGISTER = 0x03

# Initialize I2C (use '1' for newer Raspberry Pi models)
bus = smbus.SMBus(1)

#calib sensor:
al = []
zl = [] 
for i in range (1,100):

    # Read the sensor data
    a, z = read_sensor_data()
    
    # Print the sensor data
    
    al.append(a)
    zl.append(z)
    #sai so a 
    lech_a = np.mean(al)
    lech_z = np.mean(zl)
print('he')
lat0,lon0 = read_gps()
while lat0 == None : 
    lat0,lon0 = read_gps()
    time.sleep(0.1)


# data_to_send = b"a 200 200 1 1\n" # ? 
# prev_time  = time.time()
# while time.time() - prev_time < 1 :
#     arduino_ser.write(data_to_send)
#     time.sleep(0.1)
    
# time.sleep(3)
# print('hi')

# Set HMC5883L to continuous measurement mode
bus.write_byte_data(HMC5883L_ADDR, MODE_REGISTER, 0x00)
x_raw = mag.read_signed_16bit(DATA_REGISTER)
z_raw = mag.read_signed_16bit(DATA_REGISTER + 2)
y_raw = mag.read_signed_16bit(DATA_REGISTER + 4)
x_calibrated, y_calibrated, z_calibrated = mag.calibrate_magnetometer(x_raw, y_raw, z_raw)
#x_transformed, y_transformed,z_transformed = mag.transform_coordinate_system(x_calibrated,y_calibrated,z_calibrated)
theta = mag.calculate_angle(y_calibrated,x_calibrated)+ np.pi/2
theta = (theta + np.pi) % (2*np.pi) - np.pi

#theta = np.arctan2((y-y_p),(x-x_p))

X = np.zeros((4,1))
# when start X-axis of robot is 90 degree vs X-axis of the Oxy we define 

#velocity
print(theta*180/np.pi)
y_d1  = np.sin(theta)*100
x_d1  = np.cos(theta)*100

X = np.array([[0],[0],[0.1],[theta]])


start_byte = 'a'
#not_reach = True

prev_time = time.time()

waypoints = [(x_d1,y_d1)]
# with open('way_point.txt','r') as file : 
#     for line in file : 
#         fields = line.strip().split(',')
#         lat = float(fields[0])/180*np.pi
#         long = float(fields[1])/180*np.pi
#         x,y = gps_to_x_y(lat0,lon0,lat,long)
#         waypoints.append((x,y))

waypoints.append((0,0))

print(waypoints)
prev_time_gps = time.time()
for waypoint in waypoints:
    x_d = waypoint[0]
    y_d = waypoint[1]
    print(x_d,y_d)
    with open('record.txt','w') as file : 
        while True :
            ax,w = read_sensor_data() 
            ax = ax- lech_a 
            w = w - lech_z 
            current_time = time.time()
            deltaT = current_time - prev_time if prev_time else 0
            prev_time = current_time
            F = get_F(X[3][0], deltaT, w)
            X, P = kalman_predict(X, P, Qk, F, ax, w, deltaT)
            X[3][0] =  (X[3][0] + np.pi) % (2*np.pi) - np.pi

            deltaT_gps = time.time() - prev_time_gps  
            
            if deltaT_gps >= 1:
                try: 
                    lat,lon = read_gps()
                    if lat == None or lon == None: 
                        continue
                    print("update")
                    x,y = gps_to_x_y(lat0,lon0,lat,lon)
                    v = np.sqrt((x-x_p)**2 +(y- y_p)**2)/deltaT_gps
                    x_raw = mag.read_signed_16bit(DATA_REGISTER)
                    z_raw = mag.read_signed_16bit(DATA_REGISTER + 2)
                    y_raw = mag.read_signed_16bit(DATA_REGISTER + 4)
                    x_calibrated, y_calibrated, z_calibrated = mag.calibrate_magnetometer(x_raw, y_raw, z_raw)
                    #x_transformed, y_transformed,z_transformed = mag.transform_coordinate_system(x_calibrated,y_calibrated,z_calibrated)
                    theta = mag.calculate_angle(y_calibrated,x_calibrated)+ np.pi/2
                    #theta = np.arctan2((y-y_p),(x-x_p))
                    Z = np.array([[x], [y], [0.2], [theta]])
                    X, P = kalman_update(X, P, H, Rk, Z)
                    prev_time_gps = time.time() 
                    x_p = x
                    y_p = y
                except serial.serialutil.SerialException:
                    prev_time_gps = time.time()
                    
                    gps_ser.close()
                    
                    gps_ser.open()
                    
                    continue
            
            
            file.write(str((X[0][0],X[1][0],X[3][0]))+ '\n')
            pwmr,pwml,dirr,dirl = control.control(X[0][0],X[1][0],X[2][0],X[3][0],x_d,y_d)
            if distance_cal(X[0][0],X[1][0],x_d,y_d) < 1 :
                break 
            #add code send data
           
            command = f"{start_byte} {pwml} {pwmr} {dirl} {dirr}\n"
            arduino_ser.write(command.encode('ascii')) 
            time.sleep(0.1)

# add code dung xe 
prev_time  = time.time()                                           
while time.time()- prev_time <1:
	arduino_ser.write(b'a 0 0 1 1\n')
	time.sleep(0.2)


print('done')
