import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from usefulFuncs import *
from classDefs import *
import time
from brain import *

###---- Class that contains path information -----####
# class Path:

#     def __init__(brain):

#         # brain.a_el1 = 8; brain.a_el2 = 4  # major axis
#         # brain.b_el1 = 6; brain.b_el2 = 2  # minor axis
#         # brain.n_spacing = 400

#         # brain.x_d = np.concatenate( (np.linspace(-brain.a_el1, brain.a_el1, brain.n_spacing) ,\
#         # np.linspace(brain.a_el1, -brain.a_el1, brain.n_spacing) ) , axis = 0 )      #needs to be a 1D array
#         # brain.y_d = np.concatenate( (brain.b_el1*np.sqrt(1 - np.square(brain.x_d[0:brain.n_spacing]/brain.a_el1 ) ) , \
#         # -brain.b_el1*np.sqrt(1 - np.square(brain.x_d[brain.n_spacing:]/brain.a_el1 ) ) ) , axis=0)

#         brain.n_spacing = 100
#         brain.x_d = np.concatenate( (np.linspace(-brain.a_el1, brain.a_el1, brain.n_spacing) ,\
#          np.linspace(brain.a_el1, -brain.a_el1, brain.n_spacing) ) , axis = 0 )     #needs to be a 1D array
#         brain.y_d = np.zeros(2*n_spacing)


# ###---- SnowBot Class -----####
# class SnowBot:

#     def __init__(brain, x=0.0 , y=0.0 , theta=0.0):

#         brain.w_l = 0.0
#         brain.w_r = 0.0

#         # geometric parameters
#         brain.r = 0.13 #radius of wheel
#         brain.w_wheel = 0.076 #wheel width
#         brain.c = 0.534 #lateral width
#         brain.x_p = 0.85 #plow offset

#         brain.state = brain.State(x,y,theta)

#         brain.x0_actual = 0.3 # x_ICR

#         brain.Q_actual = np.array([[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 1e-5]])
#         brain.R_actual = np.array([[1e-2, 0, 0], [0, 1e-2, 0], [0, 0, 1e-3]])


#     ###---- State Class -----####
#     class State:

#         def __init__(brain, x=0.0 , y= 0.0 , theta=0.0):
#             brain.x = x
#             brain.y = y
#             brain.theta = theta



# ###---- Controller Class -----####
# class Controller:

#     #control parameters
#     L = 0.7  #bounding box length
#     k_p = 3.0  #gain

#     def __init__(brain, snowbot, path):

        
#         brain.dt_ctrl = 0.05
#         brain.v_bot = 0.8
#         brain.w_max = 60*np.pi*2/60.0
#         brain.block = np.floor(len(path.x_d)/10)
#         brain.search = 200  #look-ahead for search

#         # find initial gamma_prev 
#         X_plow = snowbot.state.x + snowbot.x_p * np.cos( snowbot.state.theta )
#         Y_plow = snowbot.state.y + snowbot.x_p * np.sin( snowbot.state.theta )
#         temp, brain.gamma_prev = closest_node(X_plow, Y_plow, np.array([path.x_d, path.y_d]).T )        
        

#     def control_update(brain, snowbot, path):

#         #initialise variables
#         gamma_prev = brain.gamma_prev
#         x = snowbot.state.x; y = snowbot.state.y; theta = snowbot.state.theta
#         x_p = snowbot.x_p
#         v_bot = brain.v_bot; L = brain.L; k_p=brain.k_p; search=brain.search
#         block = brain.block

#         # Nonlinear guidance law path-following controller

#         x_d_ref=Path().x_d;  y_d_ref=Path().y_d

#         # block prev path
#         x_d_ref[wrappath(np.arange(gamma_prev - block, gamma_prev), path)] = 1e8
#         y_d_ref[wrappath(np.arange(gamma_prev - block, gamma_prev), path)] = 1e8

#         # plow transform
#         X_plow = x + x_p * np.cos( theta );  Y_plow = y + x_p * np.sin( theta )

#         # block future path
#         x_d_ref[wrappath(np.arange(gamma_prev+search,gamma_prev+search+block), path)] = 1e8
#         y_d_ref[wrappath(np.arange(gamma_prev+search,gamma_prev+search+block), path)] = 1e8

#         # find d_min, gamma_min
#         d_min, gamma_min = closest_node(X_plow, Y_plow, np.array([x_d_ref, y_d_ref]).T )

#         x_d_ref2=Path().x_d;  y_d_ref2=Path().y_d

#         # find VTP
#         if d_min > L:
#             gamma_vtp = gamma_min

#         else:
#             ind = wrappath(np.arange(gamma_min-block, gamma_min), path)
#             # print(ind)
#             x_d_ref2[ind] = 1e8
#             y_d_ref2[ind] = 1e8
#             qty2 = np.abs(np.sqrt((x_d_ref2 - X_plow)**2 + (y_d_ref2 - Y_plow)**2 ) - L) 
#             gamma_vtp = np.argmin(qty2)

#         # check discontinuity
#         # if LA.norm( np.array([[ x_d_ref[wrappath(gamma_vtp,path)] ],[ y_d_ref[wrappath(gamma_vtp,path)]] ]) - \
#         #     np.array([[ x_d_ref[wrappath(gamma_vtp+1,path)] ],[ y_d_ref[wrappath(gamma_vtp+1,path)]] ])  ) >= L:

#         #     gamma_vtp= gamma_vtp+1

#         # else:
#         #     pass

#         # check atan2 defined or not
#         if (Path().y_d[wrappath(gamma_vtp, path)] - Y_plow == 0.0) and (Path().x_d[wrappath(gamma_vtp,path)] - X_plow == 0.0):
#             deltad = 0
#             print('atan2 not defined')

#         else:
#             # calculate theta_ref
#             theta_ref = np.arctan2( Path().y_d[wrappath(gamma_vtp, path)] - Y_plow , Path().x_d[wrappath(gamma_vtp,path)] - X_plow  )
#             # print(theta_ref)
#             deltad = theta - theta_ref

#         w_r = float(( v_bot - k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)
#         w_l = float(( v_bot + k_p*snowbot.c*wrap2pi(deltad) )/snowbot.r)


#         #include RPM thresholds
#         if w_r > brain.w_max:
#             w_r = brain.w_max

#         if w_r < -brain.w_max:
#             w_r = -brain.w_max

#         if w_l > brain.w_max:
#             w_l = brain.w_max

#         if w_l < -brain.w_max:
#             w_l = -brain.w_max

#         snowbot.w_r = w_r
#         snowbot.w_l = w_l

#         brain.gamma_prev = gamma_min


# ###---- Sensor Fusion Class -----####
# class SensorFusion:

#     #EKF parameters
#     Q_ekf = np.array([[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 1e-5]])
#     R_ekf = np.array([[1e-2, 0, 0], [0, 1e-2, 0], [0, 0, 1e-3]])
#     # J_w = np.eye(3)
#     x0_ekf = 0.3

#     def __init__(brain):

#         brain.P = 1e-4*np.eye(3)  #uncertainty in initial condition
#         brain.dt_ekf = 0.01

        
#     def state_update(brain, snowbot, measurement):

#         # all req parameters
#         w_l = snowbot.w_l; w_r = snowbot.w_r
#         r = snowbot.r; c = snowbot.c
#         x_prev = float(snowbot.state.x); y_prev = float(snowbot.state.y); theta_prev = float(snowbot.state.theta)
#         x_meas = float(measurement.x); y_meas = float(measurement.y); theta_meas = float(measurement.theta)
#         dt = brain.dt_ekf; Q_ekf=brain.Q_ekf; R_ekf = brain.R_ekf; x0_ekf = brain.x0_ekf
#         P_prev = brain.P # updated P at prev step

#         # concatenate states
#         prev_state = np.array([[x_prev],[y_prev],[theta_prev]])
#         measured_state = np.array([[x_meas],[y_meas],[theta_meas]])

#         # state a-priori estimate
#         a_priori = np.array([[x_prev + dt*( (w_l+w_r)*np.cos(theta_prev)*r/(2.0) + (w_r - w_l)*np.sin(theta_prev)*r*x0_ekf/(2.0*c) )] \
#         , [ y_prev + dt*( (w_l+w_r)*np.sin(theta_prev)*r/(2.0) - (w_r - w_l)*np.cos(theta_prev)*r*x0_ekf/(2.0*c) ) ],\
#         [ theta_prev + dt*(w_r - w_l)*r/(2.0*c) ]])

#         # calculate jacobian

#         # print(theta_prev)
#         F_prev = np.array( [[ 1, 0, -(w_l+w_r)*r*np.sin(theta_prev)/(2.0) + (w_r-w_l)*r*x0_ekf*np.cos(theta_prev)/(2.0*c) ],\
#         [ 0, 1, (w_l+w_r)*r*np.cos(theta_prev)/(2.0) + (w_r-w_l)*r*x0_ekf*np.sin(theta_prev)/(2.0*c) ],[ 0,0,1 ]] )

#         # a-priori P
#         # print(F_prev, P_prev)
#         P_ap = F_prev @ P_prev @ F_prev.T + Q_ekf

#         # find kalman gain matrix
#         K_kal = P_ap @ LA.inv( P_ap + R_ekf )

#         # next state estimate
#         state_est = a_priori + K_kal @ ( measured_state - a_priori )

#         #update P
#         P_update = ( np.eye(3) - K_kal ) @ P_ap

#         brain.P = P_update
#         snowbot.state.x = state_est[0,0]; snowbot.state.y = state_est[1,0]; snowbot.state.theta = state_est[2,0]; 


# ###---- Measurement state  -----####
# class Measurement:
# # in actual hardware replace snowbot by arduino serialisation
#     def __init__(brain, snowbot):

#         # x_init, y_init, theta_init = arduino.serialisation()
#         # brain.x = x_init
#         # brain.y = y_init
#         # brain.theta = theta_init
#         brain.x = snowbot.state.x + np.random.normal(0, 0.01, 1)
#         brain.y = snowbot.state.y + np.random.normal(0, 0.01, 1)
#         brain.theta = snowbot.state.theta + np.random.normal(0, 0.001, 1)

#     def measurementUpdate(brain, snowbot):

#         # x_obs, y_obs, theta_obs = arduino.serialisation()
#         brain.x = snowbot.state.x + np.random.normal(0, 0.1, 1)
#         brain.y = snowbot.state.y + np.random.normal(0, 0.1, 1)
#         brain.theta = snowbot.state.theta + np.random.normal(0, 0.001, 1)


######---- Main script -----#####
# if __name__ == '__main__':
    
#     count=0
#     snowbot = SnowBot(0, -10 ,1.57)
#     path = Path()
#     controller = Controller(snowbot, path)    
#     measurement =Measurement(snowbot)
#     sensorfusion = SensorFusion()

#     x_list = []; y_list =[]; theta_list = []; w_l_list = []; w_r_list = []
#     n_ct = 7000

#     while count <n_ct:

#         measurement.measurementUpdate(snowbot)
#         sensorfusion.state_update(snowbot, measurement)
#         controller.control_update(snowbot, path)
#         snowbot.state.x , snowbot.state.y , snowbot.state.theta = simulator(snowbot, controller)
#         x_list.append(snowbot.state.x); y_list.append(snowbot.state.y); theta_list.append(snowbot.state.theta)
#         w_l_list.append( snowbot.w_l ); w_r_list.append( snowbot.w_r )
#         count+=1

#     # print(path.x_d)
#     plt.plot(x_list, y_list)
#     plt.show()

#     plt.plot(np.arange( 0, n_ct ), w_l_list, 'g--')
#     plt.show()

#     plt.plot(np.arange( 0, n_ct ), w_r_list, 'r--')
#     plt.show()



##### New #######



##### Main script ########
if __name__ == '__main__':

    brain = Brain()
    ct=0
    x_list = []; y_list =[]; theta_list = []; w_l_list = []; w_r_list = []

    while (ct<20000):
        
        # if (Brain.checkEMButton == 1):
        #     brain.EMProc()

        # elif Brain.checkReset == 1:
        #     brain.resetProc()

        if brain.state == 'init':
            brain.initProc()

        elif brain.state == 'think':
            brain.thinkProc()

        elif brain.state == 'driveD':
            brain.driveDProc()

        elif brain.state == 'driveE':
            brain.driveEProc()

        elif brain.state == 'dump':
            brain.dumpProc()

        elif brain.state == 'goHome':
            brain.goHomeProc()

        elif brain.state == 'done':
            break

        brain.snowbot.state.x , brain.snowbot.state.y , brain.snowbot.state.theta\
             = simulator(brain.snowbot, brain.controller)

        x_list.append(brain.snowbot.state.x); y_list.append(brain.snowbot.state.y); 
        theta_list.append(brain.snowbot.state.theta)
        w_l_list.append( brain.snowbot.w_l ); w_r_list.append( brain.snowbot.w_r )

        ct+=1

plt.plot(x_list, y_list)
plt.show()

plt.plot(np.arange( 0, ct ), w_l_list, 'g--')
plt.show()

plt.plot(np.arange( 0, ct ), w_r_list, 'r--')
plt.show()