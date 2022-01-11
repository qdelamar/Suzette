#!/usr/bin/python3 -u

from mpu9250_i2c import *
from mx64 import *
from math import *
import signal
import threading
import time
from socket import socket, SOL_SOCKET, SO_REUSEADDR
import os


os.chdir("/home/pi/Suzette2") #ensure correct working dir


params=dict()
states=dict()

def init_params():
    global params, states

    #timing stuff:
    params["control_dt"] = 0.01 # that's quite bad but no need to go further for this system. The dxl and mpu communications seem quite limiting
    params["SKIP_MISSED_SAMPLES"] = 1 # skip iterations that are late of strictly more than this number of samples, i.e., 0 means don't try to recover any missed sample (where 'missed' means late of >= 1 dt)
    params["t0"]=time.time()
    states["skipped_control"]=0
    states["filtered_control_dt"]=params["control_dt"] # for perf measurement

    #control gains:
    params["KP_vel"] = 0.5 #rd/(m/s)  TODO: add a KD_vel to tighten+smooth the closed-loop
    params["f_gyroscopic"] = 0.7 #Hz
    params["KI_volant"] = 0.05 #rd/s
    params["I_volant_antiwindup"] = exp(-params["control_dt"]/1) #alpha_star->0 at this rate all the time
    params["f_lowpass"]=50 #Hz  used to smooth external loops for steering
    params["LOW_PASS"]=exp(-params["control_dt"]*2*pi*params["f_lowpass"])

    params["ITSMC_KP_volant"] = 11 #10
    params["ITSMC_KD_volant"] = 9 #7
    params["ITSMC_power_d_volant"] = 0.5
    params["ITSMC_power_p_volant"] = params["ITSMC_power_d_volant"]/(2-params["ITSMC_power_d_volant"])
    params["ITSMC_KS_volant"] = 0 #not in the original paper but found in some other paper... apparently just adds noise on this motor
    params["ITSMC_acc_volant"] = 3 #6 #rd/s²  this should be > max angular acceleration of system in the free dynamics (disturbance)

    params["ITSMC_KP_roue"] = 4
    params["ITSMC_KD_roue"] = 4
    params["ITSMC_power_d_roue"] = 0.5
    params["ITSMC_power_p_roue"] = params["ITSMC_power_d_roue"]/(2-params["ITSMC_power_d_roue"])
    params["ITSMC_KS_roue"] = 2.5 #here it seems to help a bit (maybe, not even sure)
    params["ITSMC_acc_roue"] = 2.5

    params["epsilon"] = 0.01 #smoothness of the smc sign

    states["do_control"] = 0 #wait for control enable from user (web interface)


    #system characteristics:
    params["gain_volant"] = 900 #actuation_unit/(rd/s²)
    params["gain_roue"]   = 600 #300

    params["ID_VOLANT"]=3
    params["ID_ROUE"]=2
    with dxl_lock:
        params["VMAX"] = dxl.get_velocity_limit(params["ID_VOLANT"]) # 285
    params["WMAX"] = 63*pi/30 #rd/s
    params["wheel_radius"] = 0.04 #m

    params["very_lowpass"] = exp(-params["control_dt"]/5)


    #sensors stuff:
    params["COMPL_FILTER"] = exp(-params["control_dt"]/1.1) # acc/gyro compromise: increasing tau means trusting more the gyro
    params["ZERO_alpha"] =  2.64*pi/180 # 0.041 #this is adjusted with online estimation
    params["ZERO_theta"] = -1.05*pi/180 #-0.018


    #online estimation stuff:
    params["w_est_zero"]=0.1 #1 # w for estimation of biases ZERO_alpha and ZERO_theta based on dynamics (ddalpha/sin(alpha) should be constant)
    params["w02_a"] = 110 #9.81/0.08 # initial guess, then this is estimated online
    params["w02_t"] = 800 #9.81/0.02 # same
    params["filter_w02_a"] = params["filter_w02_t"] = exp(-params["control_dt"]/0.16) # initial low pass cutoff
    params["filter_w02_freqmul"] = exp(-params["control_dt"]/5) # speed at which the cutoff frequency decreases (f*=this each iteration)
    params["max_w02_a_variation"] = params["max_w02_t_variation"] = 10 #10 # ignore outliers (but be permissive at the beginning as the value is quite unknown, then this value gets decreased)
    params["max_w02_variation_decrease"] = exp(-params["control_dt"]/15) # speed at which the outlier threshold decreases
    params["threshold_zero_volant"]=10 # torque considered null
    params["threshold_zero_roue"]=3


    #init integrators and other states:
    states["si_v"]=states["si_r"]=0
    states["v_volant"]=states["v_roue"]=0

    states["ready_to_send_v_volant"]=0
    states["ready_to_send_v_roue"]=0

    states["alpha"] = 0
    states["theta"] = 0
    states["ddalpha"]=0
    states["ddtheta"]=0
    states["ldalpha"]=0
    states["ldtheta"]=0
    states["alpha_star"] =0
    states["theta_star"] =0
    states["dalpha_star"]=0
    states["dtheta_star"]=0
    states["vel_star"]=states["dyaw_star"]=0

    states["running"] = 1 #let threads do their jobs





## Set realtime priority (not strictly necessary but it makes the code look like it does important stuff) (are you reading this? :p):
os.system("sudo chrt -avfp 70 "+str(os.getpid())) # fifo with priority 70/99  (applied to all threads)




## Dynamixel stuff:
dxl = DynamixelInterface(mx64_control_table)
dxl.open_port('/dev/ttyACM0', 115200, 2.0)

dxl_lock = threading.Lock()
init_params()

for ID in [params["ID_VOLANT"], params["ID_ROUE"]]:
    dxl.set_operating_mode(ID, operating_modes['velocity_control_mode'])



## Handle Ctrl-C:
threads = []
def stop(sig,fr):
    global states,threads
    states["running"] = 0 #threads should stop
    print("\n"*6) ##
    print("[+] Ctrl-C")
signal.signal(signal.SIGINT,stop)




## Web server stuff:
with open('control.html','rb') as nf:
    html_content=nf.read()

try:
    PORT=int(sys.argv[1])
except:
    PORT=8080
print("[+] Web server on port",PORT)

s0=socket()
s0.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
s0.bind(('0',PORT))
s0.listen(1)

def end_co(s):
    try:
        s.shutdown(2)
        s.close()
    except:
        pass

def serv():
    global states
    s0.settimeout(0.5) #to force non blocking accept
    while states["running"]:
        try:
            s,a=s0.accept()
        except Exception as e:
            if str(e)!="timed out":
                print("[!] Error while connecting:",e) ## debug
            continue
        #print("[+] Got a request from",a[0]) ## debug
        try:
            r=s.recv(1024)
        except Exception as e:
            print("[!] Cannot recv from",a[0],":",e) ## debug
            end_co(s)
            continue
        try:
            path=r.split(b' HTTP/')[0][4:].strip(b"/ ")
        except:
            path=b''
        #print("path:",path) ## debug
        if path in [b'']:
            try:
                s.send(b'HTTP/1.0 200 OK\r\nContent-Type:text/html\r\nConnection:close\r\n\r\n')
                s.send(html_content)
            except Exception as e:
                print("[!] Cannot send to",a[0],":",e) ## debug
        elif b"favicon" in path:
            pass #ignore
        elif path==b"stop":
            print("\n"*6) ##
            print("[+] Stop from web interface")
            states["running"]=0
        elif path==b"start":
            #enable motors:
            init_params() #reset all values
            with dxl_lock:
                dxl.set_torque_state(params["ID_VOLANT"], 1)
                dxl.set_torque_state(params["ID_ROUE"], 1)
            print("") #
            states["do_control"] = 1
        elif path==b"pause":
            #disable motors:
            with dxl_lock:
                dxl.set_torque_state(params["ID_VOLANT"], 0)
                dxl.set_torque_state(params["ID_ROUE"], 0)
            print("") #
            states["do_control"] = 0
        elif path==b"zero":
            states["dyaw_star"]=0
            states["vel_star"]=0
        else:
            try:
                value=float(path.split(b"/")[1])
            except:
                print("[!] bad request",path) ## debug
                end_co(s)
                continue
            if path.startswith(b"fwd/"):
                states["vel_star"]=value
            elif path.startswith(b"bwd/"):
                states["vel_star"]=-value
            elif path.startswith(b"right/"):
                states["dyaw_star"]=value
            elif path.startswith(b"left/"):
                states["dyaw_star"]=-value
        end_co(s)
    end_co(s0)
    print("[-] Web server thread end")

thread_serv = threading.Thread(target=serv)
thread_serv.setDaemon(1)
thread_serv.start()
threads.append(thread_serv)






## Control loop:
def control_loop():
    global params, states

    actual_last_t=time.time()
    sched_t=actual_last_t+params["control_dt"]
    while states["running"]:
        t=time.time()
        if t-sched_t>=0:
            sched_t+=params["control_dt"]
            while t-sched_t >= params["SKIP_MISSED_SAMPLES"]*params["control_dt"]: # skip iterations that are too late
                sched_t+=params["control_dt"]
                states["skipped_control"]+=1
            filtered_control_dt = 0.1*(t-actual_last_t)+0.9*states["filtered_control_dt"]
            actual_last_t=t

            if not states["do_control"]:
                continue

            ## let's do the control stuff now:
            ax,ay,az,wx,wy,wz = mpu6050_conv()
            states["dalpha"] = wx*pi/180
            states["dtheta"] = wy*pi/180

            states["alpha"] = (states["alpha"]+states["dalpha"]*params["control_dt"])*params["COMPL_FILTER"] + (atan2( ay,sqrt(ax**2+az**2)) + params["ZERO_alpha"])*(1-params["COMPL_FILTER"])
            states["theta"] = (states["theta"]+states["dtheta"]*params["control_dt"])*params["COMPL_FILTER"] + (atan2(-ax,sqrt(ay**2+az**2)) + params["ZERO_theta"])*(1-params["COMPL_FILTER"])

            if abs(states["alpha"])>pi/6 or abs(states["theta"])>pi/6:
                print("\n"*6) ##
                print("[!] Falling")
                #states["running"] = 0
                #disable motors:
                with dxl_lock:
                    dxl.set_torque_state(params["ID_VOLANT"], 0)
                    dxl.set_torque_state(params["ID_ROUE"], 0)
                states["do_control"] = 0


            states["ddalpha"] = states["ddalpha"]*0.8 + (1-0.8)*(states["dalpha"]-states["ldalpha"])/params["control_dt"]
            states["ddtheta"] = states["ddtheta"]*0.8 + (1-0.8)*(states["dtheta"]-states["ldtheta"])/params["control_dt"]
            states["ldalpha"]=states["dalpha"]
            states["ldtheta"]=states["dtheta"]


            #feedback on alpha to keep far from actuator saturation:
            states["alpha_star"] += params["KI_volant"]*(0-states["ready_to_send_v_volant"]/params["VMAX"])*params["control_dt"]
            states["alpha_star"] *= params["I_volant_antiwindup"] #keep close to 0

            #linear velocity control:
            lin_vel = -states["ready_to_send_v_roue"]*params["WMAX"]/params["VMAX"]*params["wheel_radius"] #m/s
            states["theta_star"] = states["theta_star"]*params["LOW_PASS"] + (1-params["LOW_PASS"])* params["KP_vel"]*(states["vel_star"]-lin_vel) #external loop

            #gyroscopic steering:
            sinusoid = sin(t*2*pi*params["f_gyroscopic"])
            states["dalpha_star"] = states["dalpha_star"]*params["LOW_PASS"] + (1-params["LOW_PASS"])* sinusoid*states["dyaw_star"]       #the sign of dyaw_star makes them
            states["dtheta_star"] = states["dtheta_star"]*params["LOW_PASS"] + (1-params["LOW_PASS"])* sinusoid*abs(states["dyaw_star"])  #either in phase or in antiphase


            #integral terminal sliding mode:
            ep_v = states["alpha"]-states["alpha_star"]
            ed_v = states["dalpha"]-states["dalpha_star"]
            sgn_ep_v = ep_v/(params["epsilon"]+abs(ep_v))
            sgn_ed_v = ed_v/(params["epsilon"]+abs(ed_v))
            u_pd_v = params["ITSMC_KP_volant"]*sgn_ep_v*abs(ep_v)**params["ITSMC_power_p_volant"] + \
                     params["ITSMC_KD_volant"]*sgn_ed_v*abs(ed_v)**params["ITSMC_power_d_volant"]
            states["si_v"] += u_pd_v*params["control_dt"]
            s_v = ed_v+states["si_v"]
            sgn_s_v = s_v/(params["epsilon"]+abs(s_v))
            u_volant = u_pd_v + \
                       params["ITSMC_KS_volant"]*s_v + \
                       params["ITSMC_acc_volant"]*sgn_s_v*abs(s_v)

            ep_r = states["theta"]-states["theta_star"]
            ed_r = states["dtheta"]-states["dtheta_star"]
            sgn_ep_r = ep_r/(params["epsilon"]+abs(ep_r))
            sgn_ed_r = ed_r/(params["epsilon"]+abs(ed_r))
            u_pd_r = params["ITSMC_KP_roue"]*sgn_ep_r*abs(ep_r)**params["ITSMC_power_p_roue"] + \
                     params["ITSMC_KD_roue"]*sgn_ed_r*abs(ed_r)**params["ITSMC_power_d_roue"]
            states["si_r"] += u_pd_r*params["control_dt"]
            s_r = ed_r+states["si_r"]
            sgn_s_r = s_r/(params["epsilon"]+abs(s_r))
            u_roue = u_pd_r + \
                     params["ITSMC_KS_roue"]*s_r + \
                     params["ITSMC_acc_roue"]*sgn_s_r*abs(s_r)


            #actuation gain:
            states["v_volant"] = params["gain_volant"]*u_volant
            states["v_roue"]   = params["gain_roue"]  *u_roue


            #estimate falling pulsation w0²:
            if t-params["t0"]>1: # let a bit of time for initial stabilization
                if states["alpha"]!=0:
                    new_w02_a = states["ddalpha"]/sin(states["alpha"]) # assuming alpha bias is small...
                    params["max_w02_a_variation"] *= params["max_w02_variation_decrease"] # ignore outliers, more and more strictly
                    if 1/(1+params["max_w02_a_variation"]) < new_w02_a/params["w02_a"] and new_w02_a/params["w02_a"] < 1+params["max_w02_a_variation"]:
                        params["w02_a"] = params["w02_a"]*params["filter_w02_a"] + (1-params["filter_w02_a"])* new_w02_a
                        params["filter_w02_a"] **= params["filter_w02_freqmul"] # ->1
                if states["theta"]!=0:
                    new_w02_t = states["ddtheta"]/sin(states["theta"])
                    params["max_w02_t_variation"] *= params["max_w02_variation_decrease"]
                    if 1/(1+params["max_w02_t_variation"]) < new_w02_t/params["w02_t"] and new_w02_t/params["w02_t"] < 1+params["max_w02_t_variation"]:
                        params["w02_t"] = params["w02_t"]*params["filter_w02_t"] + (1-params["filter_w02_t"])* new_w02_t
                        params["filter_w02_t"] **= params["filter_w02_freqmul"]

            #dxl.get_load takes too long
            if abs(states["v_volant"])<params["threshold_zero_volant"]:
                #applied torque is almost 0, so if the movement is in the right plane,
                #we should have the inverted pendulum behavior:
                # d²actual_alpha/dt²=sin(actual_alpha)*w0²   (where w0²~=g/l for point mass but it's not a point mass so w0 is something complicated with the inertia)
                #which means that we can build the following estimator:
                params["ZERO_alpha"] += params["control_dt"]*params["w_est_zero"]*(states["ddalpha"]/params["w02_a"]-sin(states["alpha"]))
                #print("dynamic estimation of ZERO_alpha:",ZERO_alpha) ## debug
            if abs(states["v_roue"])<params["threshold_zero_roue"]:
                params["ZERO_theta"] += params["control_dt"]*params["w_est_zero"]*(states["ddtheta"]/params["w02_t"]-sin(states["theta"]))
                #print("dynamic estimation of ZERO_theta:",ZERO_theta) ## debug


            #v_volant and v_roue should be angular accelerations so let's integrate them to find out the speed that we can ask to the servo-motors:
            states["ready_to_send_v_volant"] = min(max(int(states["ready_to_send_v_volant"]+states["v_volant"]*params["control_dt"]),-params["VMAX"]),params["VMAX"])
            states["ready_to_send_v_roue"]   = min(max(int(states["ready_to_send_v_roue"]  -states["v_roue"]  *params["control_dt"]),-params["VMAX"]),params["VMAX"])

            #set motors:
            with dxl_lock:
                dxl.set_goal_velocity(params["ID_VOLANT"], states["ready_to_send_v_volant"])
                dxl.set_goal_velocity(params["ID_ROUE"],   states["ready_to_send_v_roue"])

        else: #need to wait for the next schedule
            time.sleep(params["control_dt"]*0.01) # sched_yield...
    print("[-] Control thread end")


thread_control_loop = threading.Thread(target=control_loop)
thread_control_loop.setDaemon(1)
thread_control_loop.start()
threads.append(thread_control_loop)


## Main thread: waiting for ctrl-c or system stop, and show general information in the mean time
print("\n")
while states["running"]:
    print("Control dt: {:.3f} ms                                  ".format(1000*states["filtered_control_dt"]))
    print("w02_a: {:.1f}, w02_t: {:.1f}                           ".format(params["w02_a"], params["w02_t"]))
    print("ZERO_alpha: {:.3f}°, ZERO_theta: {:.3f}°               ".format(params["ZERO_alpha"]*180/pi, params["ZERO_theta"]*180/pi))
    print("alpha*: {:.3f}°, vel*: {:.2f} m/s, dyaw*: {:.2f}°      ".format(states["alpha_star"]*180/pi, states["vel_star"], states["dyaw_star"]))
    print("Skipped control samples: {}                            ".format(states["skipped_control"]))
    print("Control is {}                                          ".format(["OFF","ON "][states["do_control"]]))
    print("\033[7A")
    time.sleep(0.5)

with dxl_lock:
    dxl.set_torque_state(params["ID_VOLANT"], 0)
    dxl.set_torque_state(params["ID_ROUE"], 0)

print("[-] Joining threads")
thread_control_loop.join()
thread_serv.join()


print("[-] Finito")

