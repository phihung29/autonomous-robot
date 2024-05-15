import control  as ctr 
import magnetic as mag
import smbus
import time 
import serial 
import numpy as np

start_byte = 'a'
arduino_port = '/dev/ttyUSB0'
# Define I2C address of the HMC5883L
HMC5883L_ADDR = 0x1E

# Register addresses
MODE_REGISTER = 0x02
DATA_REGISTER = 0x03

# Initialize I2C (use '1' for newer Raspberry Pi models)
bus = smbus.SMBus(1)


try:  
    arduino_ser = serial.Serial(arduino_port, 9600,timeout = 1)
    bus.write_byte_data(HMC5883L_ADDR, MODE_REGISTER, 0x00)  
    while True: 
    
        x_raw = mag.read_signed_16bit(DATA_REGISTER)
        z_raw = mag.read_signed_16bit(DATA_REGISTER + 2)
        y_raw = mag.read_signed_16bit(DATA_REGISTER + 4)
        x_calibrated, y_calibrated, z_calibrated = mag.calibrate_magnetometer(x_raw, y_raw, z_raw)
    
        theta = mag.calculate_angle(x_calibrated,y_calibrated) 
        # phi = theta + np.pi/2 
        # print(theta*180/np.pi + 180)
        pwml,pwmr,dirl,dirr = ctr.control(0,0,theta,4,4)
        print(pwml, pwmr, dirl,dirr)
        command = f"{start_byte} {pwml} {pwmr} {dirl} {dirr}\n"
        arduino_ser.write(command.encode('ascii')) 
        #Add a delay before next reading
        
        time.sleep(0.2)  # Adjust delay as needed

except KeyboardInterrupt:
    bus.close()