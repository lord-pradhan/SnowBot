import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from usefulFuncs import *

###---- Class that contains path information -----####
class Path:

    def __init__(self):

        self.a_el1 = 8; self.a_el2 = 4  # major axis
        self.b_el1 = 6; self.b_el2 = 2  # minor axis
        self.n_spacing = 400

        self.x_d = np.concatenate( (np.linspace(-self.a_el1, self.a_el1, self.n_spacing) ,\
        np.linspace(self.a_el1, -self.a_el1, self.n_spacing) ) , axis = 0 )      #needs to be a 1D array
        self.y_d = np.concatenate( (self.b_el1*np.sqrt(1 - np.square(self.x_d[0:self.n_spacing]/self.a_el1 ) ) , \
        -self.b_el1*np.sqrt(1 - np.square(self.x_d[self.n_spacing:]/self.a_el1 ) ) ) , axis=0)


###---- SnowBot Class -----####
class SnowBot:

    def __init__(self, x=0.0 , y=0.0 , theta=0.0):

        self.w_l = 0.0
        self.w_r = 0.0

        # geometric parameters
        self.r = 0.13 #radius of wheel
        self.w_wheel = 0.076 #wheel width
        self.c = 0.534 #lateral width
        self.x_p = 0.85 #plow offset

        self.state = self.State(x,y,theta)

        self.x0_actual = 0.3 # x_ICR

        self.Q_actual = np.array([[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 1e-5]])
        self.R_actual = np.array([[1e-2, 0, 0], [0, 1e-2, 0], [0, 0, 1e-3]])


    ###---- State Class -----####
    class State:

        def __init__(self, x=0.0 , y= 0.0 , theta=0.0):
            self.x = x
            self.y = y
            self.theta = theta



###---- Controller Class -----####
class Controller:

    #control parameters
    L = 0.7  #bounding box length
    k_p = 3.0  #gain

    def __init__(self, snowbot, path):

        
        self.dt_ctrl = 0.05
        self.v_bot = 0.8
        self.w_max = 60*np.pi*2/60.0
        self.block = np.floor(len(path.x_d)/10)
        self.search = 200  #look-ahead for search

        # find initial gamma_prev 
        X_plow = snowbot.state.x + snowbot.x_p * np.cos( snowbot.state.theta )
        Y_plow = snowbot.state.y + snowbot.x_p * np.sin( snowbot.state.theta )
        temp, self.gamma_prev = closest_node(X_plow, Y_plow, np.array([path.x_d, path.y_d]).T )        
        

    def control_update(self, snowbot, path):

        #initialise variables
        gamma_prev = self.gamma_prev
        x = snowbot.state.x; y = snowbot.state.y; theta = snowbot.state.theta
        x_p = snowbot.x_p
        v_bot = self.v_bot; L = self.L; k_p=self.k_p; search=self.search
        block = self.block

        # Nonlinear guidance law path-following controller

        x_d_ref=Path().x_d;  y_d_ref=Path().y_d

        # block prev path
        x_d_ref[wrappath(np.arange(gamma_prev - block, gamma_prev), path)] = 1e8
        y_d_ref[wrappath(np.arange(gamma_prev - block, gamma_prev), path)] = 1e8

        # plow transform
        X_plow = x + x_p * np.cos( theta );  Y_plow = y + x_p * np.sin( theta )

        # block future path
        x_d_ref[wrappath(np.arange(gamma_prev+search,gamma_prev+search+block), path)] = 1e8
        y_d_ref[wrappath(np.arange(gamma_prev+search,gamma_prev+search+block), path)] = 1e8

        # find d_min, gamma_min
        d_min, gamma_min = closest_node(X_plow, Y_plow, np.array([x_d_ref, y_d_ref]).T )

        x_d_ref2=Path().x_d;  y_d_ref2=Path().y_d

        # find VTP
        if d_min > L:
            gamma_vtp = gamma_min

        else:
            ind = wrappath(np.arange(gamma_min-block, gamma_min), path)
            print(ind)
            x_d_ref2[ind] = 1e8
            y_d_ref2[ind] = 1e8
            qty2 = np.abs(np.sqrt((x_d_ref2 - X_plow)**2 + (y_d_ref2 - Y_plow)**2 ) - L) 
            gamma_vtp = np.argmin(qty2)

        # check discontinuity
        # if LA.norm( np.array([[ x_d_ref[wrappath(gamma_vtp,path)] ],[ y_d_ref[wrappath(gamma_vtp,path)]] ]) - \
        #     np.array([[ x_d_ref[wrappath(gamma_vtp+1,path)] ],[ y_d_ref[wrappath(gamma_vtp+1,path)]] ])  ) >= L:

        #     gamma_vtp= gamma_vtp+1

        # else:
        #     pass

        # check atan2 defined or not
        if (Path().y_d[wrappath(gamma_vtp, path)] - Y_plow == 0.0) and (Path().x_d[wrappath(gamma_vtp,path)] - X_plow == 0.0):
            deltad = 0
            print('atan2 not defined')

        else:
            # calculate theta_ref
            theta_ref = np.arctan2( Path().y_d[wrappath(gamma_vtp, path)] - Y_plow , Path().x_d[wrappath(gamma_vtp,path)] - X_plow  )
            # print(theta_ref)
            deltad = theta - theta_ref

        w_r = float(( v_bot - k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)
        w_l = float(( v_bot + k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)


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

        self.gamma_prev = gamma_min
        print(gamma_vtp)
        print(np.array_equal(path.x_d , x_d_ref))


###---- Sensor Fusion Class -----####
class SensorFusion:

    #EKF parameters
    Q_ekf = np.array([[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 1e-5]])
    R_ekf = np.array([[1e-2, 0, 0], [0, 1e-2, 0], [0, 0, 1e-3]])
    # J_w = np.eye(3)
    x0_ekf = 0.3

    def __init__(self):

        self.P = 1e-4*np.eye(3)  #uncertainty in initial condition
        self.dt_ekf = 0.01

        
    def state_update(self, snowbot, measurement):

        # all req parameters
        w_l = snowbot.w_l; w_r = snowbot.w_r
        r = snowbot.r; c = snowbot.c
        x_prev = snowbot.state.x; y_prev = snowbot.state.y; theta_prev = snowbot.state.theta
        x_meas = measurement.x; y_meas = measurement.y; theta_meas = measurement.theta
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
        F_prev = np.array( [[ 1, 0, -(w_l+w_r)*r*np.sin(theta_prev)/(2.0) + (w_r-w_l)*r*x0_ekf*np.cos(theta_prev)/(2.0*c) ],\
        [ 0, 1, (w_l+w_r)*r*np.cos(theta_prev)/(2.0) + (w_r-w_l)*r*x0_ekf*np.sin(theta_prev)/(2.0*c) ],[ 0,0,1 ]] )

        # a-priori P
        print(F_prev, P_prev)
        P_ap = F_prev @ P_prev @ F_prev.T + Q_ekf

        # find kalman gain matrix
        K_kal = P_ap @ LA.inv( P_ap + R_ekf )

        # next state estimate
        state_est = a_priori + K_kal @ ( measured_state - a_priori )

        #update P
        P_update = ( np.eye(3) - K_kal ) @ P_ap

        self.P = P_update
        snowbot.state.x = state_est[0,0]; snowbot.state.y = state_est[1,0]; snowbot.state.theta = state_est[2,0]; 


###---- Measurement state  -----####
class Measurement:
# in actual hardware replace snowbot by arduino serialisation
    def __init__(self, snowbot):

        # x_init, y_init, theta_init = arduino.serialisation()
        # self.x = x_init
        # self.y = y_init
        # self.theta = theta_init
        self.x = snowbot.state.x + np.random.normal(0, 0.001, 1)
        self.y = snowbot.state.y + np.random.normal(0, 0.001, 1)
        self.theta = snowbot.state.theta + np.random.normal(0, 0.001, 1)

    def measurementUpdate(self, snowbot):

        # x_obs, y_obs, theta_obs = arduino.serialisation()
        self.x = snowbot.state.x + np.random.normal(0, 0.001, 1)
        self.y = snowbot.state.y + np.random.normal(0, 0.001, 1)
        self.theta = snowbot.state.theta + np.random.normal(0, 0.001, 1)


######---- Main script -----#####
if __name__ == '__main__':
    
    count=0
    snowbot = SnowBot(0, -10 ,0.0)
    path = Path()
    controller = Controller(snowbot, path)    
    # measurement =Measurement(snowbot)
    # sensorfusion = SensorFusion()

    x_list = []; y_list =[]; theta_list = []; w_l_list = []; w_r_list = []
    n_ct = 700

    while count <n_ct:
        # measurement.measurementUpdate(snowbot)
        # sensorfusion.state_update(snowbot, measurement)
        controller.control_update(snowbot, path)
        snowbot.state.x , snowbot.state.y , snowbot.state.theta  = simulator(snowbot, controller)
        x_list.append(snowbot.state.x); y_list.append(snowbot.state.y); theta_list.append(snowbot.state.theta)
        w_l_list.append( snowbot.w_l ); w_r_list.append( snowbot.w_r )
        count+=1

    # print(path.x_d)
    plt.plot(x_list, y_list)
    plt.show()

    plt.plot(np.arange( 0, n_ct ), w_l_list, 'g--')
    plt.show()

    plt.plot(np.arange( 0, n_ct ), w_r_list, 'r--')
    plt.show()