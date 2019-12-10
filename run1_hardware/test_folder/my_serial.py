import serial

ser = serial.Serial(port='/dev/ttyACM0', baudrate = 115200, timeout=1)

x =0; y=0; z=0; conf=0; flag=0

##while True:
##	# print("Hello before")
##	inData = ser.readline().decode('ascii')
##	inData = inData.replace('\r\n',"")
##	# print(type(inData))
##	if 'Init' not in inData:
##		print(inData)
##		data = inData.split(',')
##		if len(data)==5:
##			# print(len(data))
##			# print(data)
##			x = float(data[0])
##			y = float(data[1])
##			z = float(data[2])
##			conf = float(data[3])
##			flag = int(data[4])
##			print("x-",x, "Y-",y, "Z-",z, "CONF-", conf, "FLAG-", flag)
##		# print("Hello after \n")

while(1):
                    
                inData = ser.readline().decode('ascii')
                inData = inData.replace('\r\n',"")
##                print(inData)
                if 'Init' not in inData:
                    # print(inData)
                    data = inData.split(',')
                    if len(data)==5:
                        # print(len(data))
                        # print(data)
                        x = float(data[0])
                        y = float(data[1])
                        z = float(data[2])
                        conf = float(data[3])
                        flag = int(data[4])
                        break
                        
                    else:
##                        x = self.x; y = self.y; z = self.z; conf = self.conf; flag = self.flag
                        continue
                else:
                    continue
##                x = self.x; y = self.y; z = self.z; conf = self.conf; flag = self.flag

print(x,y,z,conf,flag)
