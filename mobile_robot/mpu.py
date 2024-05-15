import mpu6050
import time
import numpy as np

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)

# Define a function to read the sensor data
def read_sensor_data():
    # Read the accelerometer values
    accelerometer_data = mpu6050.get_accel_data()

    # Read the gyroscope values
    gyroscope_data = mpu6050.get_gyro_data()

    # Read temp
    temperature = mpu6050.get_temp()

    return accelerometer_data, gyroscope_data, temperature

# Start a while loop to continuously read the sensor data
al = []
zl = []
for i in range (1,100):

    # Read the sensor data
    accelerometer_data, gyroscope_data, temperature = read_sensor_data()
    a = accelerometer_data['x'] +2.5706712512083723 - 2.123801975689512
    z = (gyroscope_data['z'] - 11.17403038013725)/180*np.pi + 0.1979318300528722
    # Print the sensor data
    print("Accelerometer data:", a) 
    print("Gyroscope data:", z)
    al.append(a)
    zl.append(z)
     
print(np.mean(al))
print(np.mean(zl))
    
    


    
