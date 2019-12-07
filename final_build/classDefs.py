import numpy as np
import numpy.linalg as LA
from usefulFuncs import *
from DriveArduino import *
from MPU6050 import *


###---- SnowBot Class -----####
class SnowBot:

    def __init__(self, measurement):

        self.w_l = 0.0
        self.w_r = 0.0

        self.plow = 0

        # geometric parameters
        self.r = 0.13 #radius of wheel
        self.w_wheel = 0.076 #wheel width
        self.c = 0.534 #lateral width
        self.x_p = 0.82 #plow offset

        self.state = self.State(measurement.x , measurement.y , measurement.theta)

        # self.x0_actual = 0.3 # x_ICR

        # self.Q_actual = np.array([[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 1e-5]])
        # self.R_actual = np.array([[1e-2, 0, 0], [0, 1e-2, 0], [0, 0, 1e-3]])

        self.home_x = -10; self.home_y = 0


    ###---- State Class -----####
    class State:

        def __init__(self, x=0.0 , y= 0.0 , theta=0.0):
            self.x = x
            self.y = y
            self.theta = theta


###---- Controller Class -----####
class Controller:

    def __init__(self, snowbot, path, mark):

        #control parameters
        self.L = 0.4  #bounding box length
        self.k_p = 3.0  #gain
        # self.dt_ctrl = 0.05
        self.v_target = 0.3
        self.w_max = 45*np.pi*2/60.0
        self.v_bot = min(self.w_max*snowbot.r, self.v_target)
        self.block = np.floor(len(path.x_d)/10)
        self.search = np.floor(len(path.x_d)/8)  #look-ahead for search

        self.x_track = 0; self.y_track = 0

        # find initial gamma_prev 
        X_plow = snowbot.state.x + snowbot.x_p * np.cos( snowbot.state.theta )
        Y_plow = snowbot.state.y + snowbot.x_p * np.sin( snowbot.state.theta )
        temp, self.gamma_prev = closest_node(X_plow, Y_plow, np.array([path.x_d, path.y_d]).T )

        #check if bot is inside bounding box
        if temp>self.L:
            self.in_flag = 0
        else:
            self.in_flag = 1

        # initialise flag for dump/home
        self.dumped_flag=0
        self.home_flag = 0
        

    def control_update(self, snowbot, path, mark, mode = 0):

        x = snowbot.state.x; y = snowbot.state.y; theta = snowbot.state.theta
        x_p = snowbot.x_p
        v_bot = self.v_bot
        # plow transform
        X_plow = x + x_p * np.cos( theta );  Y_plow = y + x_p * np.sin( theta )

        if mode ==0: # normal operation

            #initialise variables
            gamma_prev = self.gamma_prev
            
            L = self.L; k_p=self.k_p; search=self.search
            block = self.block

            # Nonlinear guidance law path-following controller
            x_d_ref=Path(snowbot, mark).x_d;  y_d_ref=Path(snowbot, mark).y_d

            # block prev path
            x_d_ref[wrappath(np.arange(gamma_prev - block, gamma_prev), path)] = 1e8
            y_d_ref[wrappath(np.arange(gamma_prev - block, gamma_prev), path)] = 1e8

            # block future path
            x_d_ref[wrappath(np.arange(gamma_prev+search,gamma_prev+search+block), path)] = 1e8
            y_d_ref[wrappath(np.arange(gamma_prev+search,gamma_prev+search+block), path)] = 1e8

            # find d_min, gamma_min
            d_min, gamma_min = closest_node(X_plow, Y_plow, np.array([x_d_ref, y_d_ref]).T )

            x_d_ref2=Path(snowbot, mark).x_d;  y_d_ref2=Path(snowbot, mark).y_d

            # find VTP
            if d_min > L:
                gamma_vtp = gamma_min
                self.in_flag = 1

            else:
                ind = wrappath(np.arange(gamma_min-block, gamma_min), path)
                # print(ind)
                x_d_ref2[ind] = 1e8
                y_d_ref2[ind] = 1e8
                qty2 = np.abs(np.sqrt((x_d_ref2 - X_plow)**2 + (y_d_ref2 - Y_plow)**2 ) - L) 
                gamma_vtp = np.argmin(qty2)
                self.in_flag = 0

            # check discontinuity
            # if LA.norm( np.array([[ x_d_ref[wrappath(gamma_vtp,path)] ],[ y_d_ref[wrappath(gamma_vtp,path)]] ]) - \
            #     np.array([[ x_d_ref[wrappath(gamma_vtp+1,path)] ],[ y_d_ref[wrappath(gamma_vtp+1,path)]] ])  ) >= L:

            #     gamma_vtp= gamma_vtp+1

            # else:
            #     pass

            # check atan2 defined or not
            if (Path(snowbot, mark).y_d[wrappath(gamma_vtp, path)] - Y_plow == 0.0) \
            and (Path(snowbot, mark).x_d[wrappath(gamma_vtp,path)] - X_plow == 0.0):
                deltad = 0
                print('atan2 not defined')

            else:
                # calculate theta_ref
                theta_ref = np.arctan2( Path(snowbot, mark).y_d[wrappath(gamma_vtp, path)] \
                    - Y_plow , Path(snowbot, mark).x_d[wrappath(gamma_vtp,path)] - X_plow  )
                # print(theta_ref)
                deltad = theta - theta_ref

            w_r = float(( v_bot - k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)
            w_l = float(( v_bot + k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)

            self.gamma_prev = gamma_min

        # for dump manouver
        elif mode ==1:
            L_dump = 0.05

            x_d_ref = Path(snowbot, mark=0).x_d
            y_d_ref = Path(snowbot, mark=0).y_d

            d_min, gamma_min = closest_node(X_plow, Y_plow, np.array([x_d_ref, y_d_ref]).T )

            if d_min > L_dump:
                gamma_vtp = gamma_min

                if (Path(snowbot, mark).y_d[gamma_vtp] - Y_plow == 0.0) and \
                (Path(snowbot, mark).x_d[gamma_vtp] - X_plow == 0.0):
                    deltad = 0
                    print('atan2 not defined')

                else:
                    theta_ref = np.arctan2( Path(snowbot, mark).y_d[gamma_vtp] - Y_plow\
                     , Path(snowbot, mark).x_d[gamma_vtp] - X_plow )
                    deltad = theta - theta_ref

                w_r = float(( v_bot - k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)
                w_l = float(( v_bot + k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)

            else:
                self.dumped_flag = 1
                w_r = 0
                w_l = 0

        # go home manouver
        else:
            dist = np.array([X_plow, Y_plow]).T - np.array([snowbot.home_x,snowbot.home_y ]).T
            d_home = np.sum(np ** 2, axis=1)
            if (d_home > 0.1):
                theta_ref = np.arctan2( y_d[gamma_vtp] - Y_plow , x_d[gamma_vtp] - X_plow )
                deltad = theta - theta_ref
                w_r = float(( v_bot - k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)
                w_l = float(( v_bot + k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)

            else:
                self.home_flag=1
                w_r = 0
                w_l = 0


        #include RPM thresholds
        if w_r > self.w_max:
            w_r = self.w_max

        if w_r < -self.w_max:
            w_r = -self.w_max

        if w_l > self.w_max:
            w_l = self.w_max

        if w_l < -self.w_max:
            w_l = -self.w_max

        snowbot.w_r = w_r
        snowbot.w_l = w_l            


####---- Sensor Fusion Class -----####
class SensorFusion:

    #EKF parameters
    Q_ekf = np.array([[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 1e-3]])
    R_ekf = np.array([[2e-2, 0, 0], [0, 2e-2, 0], [0, 0, 1e-1]])
    # J_w = np.eye(3)
    x0_ekf = 0.3

    def __init__(self):

        self.P = 1e-6*np.eye(3)  #uncertainty in initial condition
        self.dt_ekf = 0.3

        
    def state_update(self, snowbot, measurement):

        # all req parameters
        w_l = snowbot.w_l; w_r = snowbot.w_r
        r = snowbot.r; c = snowbot.c
        x_prev = float(snowbot.state.x); y_prev = float(snowbot.state.y); theta_prev = float(snowbot.state.theta)
        x_meas = float(measurement.x); y_meas = float(measurement.y); theta_meas = float(measurement.theta)
        dt = self.dt_ekf; Q_ekf=self.Q_ekf; R_ekf = self.R_ekf; x0_ekf = self.x0_ekf
        P_prev = self.P # updated P at prev step

        # concatenate states
        prev_state = np.array([[x_prev],[y_prev],[theta_prev]])
        measured_state = np.array([[x_meas],[y_meas],[theta_meas]])

        # state a-priori estimate
        a_priori = np.array([[x_prev + dt*( (w_l+w_r)*np.cos(theta_prev)*r/(2.0) + (w_r - w_l)*np.sin(theta_prev)*r*x0_ekf/(2.0*c) )] \
        , [ y_prev + dt*( (w_l+w_r)*np.sin(theta_prev)*r/(2.0) - (w_r - w_l)*np.cos(theta_prev)*r*x0_ekf/(2.0*c) ) ],\
        [ theta_prev + dt*(w_r - w_l)*r/(2.0*c) ]])

        # calculate jacobian

        # print(theta_prev)
        F_prev = np.array( [[ 1, 0, -(w_l+w_r)*r*np.sin(theta_prev)/(2.0) + (w_r-w_l)*r*x0_ekf*np.cos(theta_prev)/(2.0*c) ],\
        [ 0, 1, (w_l+w_r)*r*np.cos(theta_prev)/(2.0) + (w_r-w_l)*r*x0_ekf*np.sin(theta_prev)/(2.0*c) ],[ 0,0,1 ]] )

        # a-priori P
        # print(F_prev, P_prev)
        P_ap = F_prev @ P_prev @ F_prev.T + Q_ekf

        # find kalman gain matrix
        K_kal = P_ap @ LA.inv( P_ap + R_ekf )

        # next state estimate
        state_est = a_priori + K_kal @ ( measured_state - a_priori )

        #update P
        P_update = ( np.eye(3) - K_kal ) @ P_ap

        self.P = P_update
        snowbot.state.x = state_est[0,0]; snowbot.state.y = state_est[1,0]; snowbot.state.theta = wrap2pi(state_est[2,0]); 


###---- Class that contains path information -----####
class Path:

    def __init__(self, snowbot, mark=1):

        self.a_boundary = 8; self.b_boundary = 6

        self.a_el1 = self.a_boundary - mark*snowbot.c;  # major axis
        self.b_el1 = self.b_boundary - mark*snowbot.c;  # minor axis

        self.n_spacing = 200

        self.x_d = np.concatenate( (np.linspace(-self.a_el1, self.a_el1, self.n_spacing) ,\
        np.linspace(self.a_el1, -self.a_el1, self.n_spacing) ) , axis = 0 )      #needs to be a 1D array
        self.y_d = np.concatenate( (self.b_el1*np.sqrt(1 - np.square(self.x_d[0:self.n_spacing]/self.a_el1 ) ) , \
        -self.b_el1*np.sqrt(1 - np.square(self.x_d[self.n_spacing:]/self.a_el1 ) ) ) , axis=0)

        self.gamma_dump = np.arange(250,300)

        if mark==0:
            self.x_d = self.x_d[gamma_dump]
            self.y_d = self.y_d[gamma_dump]


###---- Measurement state  -----####
class Measurement:

	def __init__(self, arduino, mpu):

		self.x, self.y = arduino.getLocn()
        self.theta = mpu.getYaw()

	def measurementUpdate(self, arduino, mpu):

		self.x, self.y = arduino.getLocn()
        self.theta = mpu.getYaw()


###---- Arduino (all info from Arduino)  -----####
class Arduino:

	def __init__(self, ser):
		
		self.x = 0.0; self.y = 0.0; self.z = 0.0; self.conf = 0; self.flag = 0
        ser.reset_input_buffer()
        self.serialisation(ser)

	def getLocn(self):

        return self.x, self.y #, self.theta

	def serialisation(self, serial_input):
        """decode serial input"""
        while(1):
            ser.reset_input_buffer()
            inData = ser.readline().decode('ascii')
            inData = inData.replace('\r\n',"")
            if 'Init' not in inData:
                # print(inData)
                data = inData.split(',')
                if len(data)==5:

                    x = float(data[0])
                    y = float(data[1])
                    z = float(data[2])
                    conf = float(data[3])
                    flag = int(data[4])
                    break
                    
                else:
                    continue
            else:
                continue

        self.x = x; self.y = y; self.z = z; self.conf = conf; self.flag = flag