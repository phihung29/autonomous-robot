import serial 
import time
arduino_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
# banh trai la dang sau , banh phai dang truoc 


	# print('input pwm, dir? \n ')
	# command = input()

pwml = 200 
pwmr = 200 
dirr = 1
dirl = 1 
start_byte = 'a'
command = f"{start_byte} {pwmr} {pwmr} {dirr} {dirl}\n"
print(command)
starttime = time.time()

# while True:
# 	arduino_port.write(command.encode('ascii'))
# 	print(command)
# 	time.sleep(1)	
# 	arduino_port.write(b"a 0 0 1 1\n")
# 	print('stop')

# 	time.sleep(1)
data_to_send = b"a 140 140 1 1\n" 
while time.time() - starttime < 1 :
    arduino_port.write(data_to_send)
    time.sleep(0.1)
time.sleep(2)
arduino_port.write(b'a 0 0 1 1\n')
print('stop')
