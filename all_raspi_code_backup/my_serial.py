import serial
import time
import math

##ser = serial.Serial(port='/dev/ttyS0', baudrate = 115200, timeout=1)
ser = serial.Serial(port='/dev/ttyACM0', baudrate = 115200, timeout=1)

##b = [2.475, -11.775, -25.725]
##s = [1.045850527, 0.935219352, 1.026090868]
##b = [10.95, 24.15, -38.625]
##s = [1.03018018, 0.870243531, 1.136115251]
b = [-19.575, 16.65]
s = [0.963556851, 1.039308176]

while True:
##    print("Hello before")
    try:
        ser.reset_input_buffer()
        inData = ser.readline().decode('ascii')
    except UnicodeDecodeError:
##        print('Data not parsable')
        continue
    inData = inData.replace('\r\n',"")
    # print(type(inData))
    if 'Init' not in inData:
	# print(inData)
        data = inData.split(',')
##        if len(data)==6:
	    # print(len(data))
	    # print(data)
        try:
##            gx = float(data[0])
##            gy = float(data[1])
##            gz = float(data[2])
            mx = float(data[3])
            my = float(data[4])
            mz = float(data[5])
            mxf = (mx - b[0]) * s[0]
            myf = (my - b[1]) * s[1]
##            mzf = (mz - b[2]) * s[2]
            angle_deg = math.degrees(math.atan2(myf, mxf))
##            angle_deg = math.degrees(math.atan2(-1*my, mx))
            print(angle_deg)
##            print(mx, ',', my, ',', mz)
##            print(gx, ',' ,gy, ',', gz, ',', mx, ',', my, ',', mz)
        except:
            continue
##            print('Couldn\'t convert to float.')
##            print("gyro x- ", gx, "gyro y- ", gy, "gyro z- ", gz)
##            print("mag x-  ", mx, "mag y-  ", my, "mag z-  ", mz, "\n")
##            time.sleep(0.2)
	# print("Hello after \n")
