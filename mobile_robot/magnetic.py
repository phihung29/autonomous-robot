import smbus
import math
import time

# Define I2C address of the HMC5883L
HMC5883L_ADDR = 0x1E

# Register addresses
MODE_REGISTER = 0x02
DATA_REGISTER = 0x03

# Initialize I2C (use '1' for newer Raspberry Pi models)
bus = smbus.SMBus(1)

# Function to read signed 16-bit data
def read_signed_16bit(register):
    high = bus.read_byte_data(HMC5883L_ADDR, register)
    low = bus.read_byte_data(HMC5883L_ADDR, register + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        return -((65535 - value) + 1)
    else:
        return value

# Function to calibrate the magnetometer
def calibrate_magnetometer(x, y, z):
    # Offset values obtained from calibration
    x_offset = 113
    y_offset = 236
    z_offset = 0
    x_calibrated = x - x_offset
    y_calibrated = y - y_offset
    z_calibrated = z - z_offset
    return x_calibrated, y_calibrated, z_calibrated

def calculate_angle(x,y):
    angle_rad = math.atan2(y,x) 
    angle_rad = (angle_rad+ math.pi) % (2*math.pi) - math.pi
    return angle_rad


# valuex = []
# valuey= [] 
# try:  
    
#     bus.write_byte_data(HMC5883L_ADDR, MODE_REGISTER, 0x00)  
#     while True: 
    
#         x_raw = read_signed_16bit(DATA_REGISTER)
#         z_raw = read_signed_16bit(DATA_REGISTER + 2)
#         y_raw = read_signed_16bit(DATA_REGISTER + 4)
#         x_calibrated, y_calibrated, z_calibrated = calibrate_magnetometer(x_raw, y_raw, z_raw)
#         # x_transformed, y_transformed,z_transformed = transform_coordinate_system(x_calibrated,y_calibrated,z_calibrated)
#         x_calibrated += 155.5
#         y_calibrated += 100.5
        
#         theta = calculate_angle(x_calibrated,y_calibrated)*180/math.pi
#         valuex.append(x_calibrated)
#         valuey.append(y_calibrated)
#         print(theta)
#         # Add a delay before next reading
        
#         time.sleep(0.1)  # Adjust delay as needed

# except KeyboardInterrupt:
#     # Clean up GPIO
#     bus.close()
#     with open('compass_calib.txt', 'w') as file: 
#         for i in range(len(valuex)) : 
#             file.write(str(valuex[i]) + ',' +str(valuey[i])+ '\n')
#     x_offset =  (max(valuex) + min(valuex)) /2
#     y_offset =  (max(valuey)+ min(valuey))/2
#     print("offset", x_offset,y_offset)