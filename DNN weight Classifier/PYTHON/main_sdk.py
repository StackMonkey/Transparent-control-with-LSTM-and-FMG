# -*- coding: utf-8 -*-
"""
Created on Wed Aug  5 14:14:03 2020

@author: dlsgrd
"""

import sys
import time
import serial
import struct
import datetime as dt
from dataclasses import dataclass
import numpy as np
import glob
import pandas as pd
import matplotlib.pyplot as plt
import pickle
from sklearn.svm import SVC
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
import itertools
import matplotlib.cm as cm
import cv2
import os

# From https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
def get_serial_ports():
    """
    Lists serial ports.
    :return: ([str]) A list of available serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        #ports = glob.glob('/dev/tty[A-Za-z]*')
        #ports = glob.glob('/dev/[A-Za-z]*')
        ports = glob.glob('/dev/rfcomm*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    results = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            results.append(port)
        except (OSError, serial.SerialException):
            pass
    print("{}\n".format(results))
    return results

class Armband:
    def __init__(self):
        super(Armband, self).__init__()
        
        self.baud = 250000
        self.dataNumBytes = 1
        self.CONNECTED = False
        self.POST_PROCESSING_DONE = False
        self.PREDICTING = True
        self.s = serial.Serial(baudrate=self.baud,timeout=3, write_timeout=3,bytesize=8)

        self.parameters = {
            "default_sampletime": 10,
            "recording_window_length" : 6000
        }
        
        self.plots = {

            "plot_axes" : 1,
            "last_plot_time" : 0,
            "gestures" : ["open", "close", "pronation"],
            "gesture_name" : 0,
            "plot_history_time" : 3,   
            "update_plot_time" : 0.5,
            "indices_2_plot" : list([[0,1,2,3,4,5,6,7],[8,9,10],[11,12,13],[14,15,16],[17,18,19],[20]]),
            "data_info" : list(['FSRs (voltage)','Gravity (m/sec^2)','Angular Velocity (rad/sec)','Linear Acceleration (m/sec^2)','Euler Angles (deg)','Battery (%)']),
            "data_y_limit" : list([[0, 4],[-10, 10],[-200, 200],[-20, 20],[-400, 400],[0, 100]]),
            "plotting_data" : np.array([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]),
            "default_plotting_data" : np.array([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]),
            "sampling_frequency" : 1000
        }

        self.data_struct = {
            "raw": {}
        }
        
        self.headers = {
            "fsr" : ["FSR_1","FSR_2","FSR_3","FSR_4","FSR_5","FSR_6","FSR_7","FSR_8"],
            "gravity" : ["GRAVITY_X","GRAVITY_Y","GRAVITY_Z"],
            "angular_velocity" : ["ANGULAR_VELOCITY_X","ANGULAR_VELOCITY_Y","ANGULAR_VELOCITY_Z"],
            "linear_acceleration" : ["LINEAR_ACCELERATION_X","LINEAR_ACCELERATION_Y","LINEAR_ACCELERATION_Z"],
            "euler_angle" : ["EULER_ANGLE_X","EULER_ANGLE_Y","EULER_ANGLE_Z"],
            "battery" : ["Battery"],
        }
        self.headers["imu"] = self.headers["gravity"]  + self.headers["angular_velocity"] + self.headers["linear_acceleration"] + self.headers["euler_angle"]
        self.headers["full"] = self.headers["fsr"] + self.headers["imu"] + self.headers["battery"] + ["t"]

    def connect(self):
        print("Attempting connection... Please wait.")
        serial_ports = get_serial_ports() 
        for serial_port in serial_ports:
            self.s.port = serial_port
            print("Handshaking at port: {}".format(serial_port))
            try:
                self.s.open()
                self.s.write(b'0')
                time.sleep(1)
                print("Bytes inWaiting: {}".format(self.s.in_waiting))
                handshake_return = self.s.read(1)
                print("Acknowledgement recieved: {}".format(handshake_return))               
                if handshake_return == b'1':
                    print("BioX device found at port: {}".format(serial_port))
                    self.CONNECTED = True
                    print("Connected = {}".format(self.CONNECTED))
                    self.s.reset_input_buffer()
                    break
                else:
                    self.s.close()
                    print("no device")    
            except:
                if self.s.is_open:
                    self.s.close()
                continue

    def calibrate(self):
        print("Calibrating... Close fist.")        
        time.sleep(0.1)
        self.s.write(b'5')
        time.sleep(15)
        print("Calibration finished.")


    def reset_gain(self):        
        self.s.write(b'4')
        print("Gain reset.")


    def start_transmission(self):        
        self.s.write(b'2')
        time.sleep(1)
        #print("Transmission started...")


    def stop_transmission(self):
               
        self.s.write(b'3')
        time.sleep(1) 
        #print("Transmission stopped.")
        while self.s.in_waiting > 0:
            self.s.read(self.s.in_waiting)
            time.sleep(1)


    def close_connection(self):        
        self.s.write(b'6')
        self.s.flushInput()
        self.s.close()
        self.s.__del__()
        print("Disconnected")


    def reset_session(self):        
        self.POST_PROCESSING_DONE = False
        self.data_struct = {
            "raw": {},
            "raw_cut": {},
            "cut_rms": {},
            "svm_data": pd.DataFrame,  
            "testing_data": []
        }


    def get_sample(self):        
        data_buffer = list()
        data = self.s.read(212)
        for i in range(20):
            data_decoded = list()
            if len(data) == 212:
                for j in range(8):
                    dataDecode = data[(i*8)+(j*1) : (i*8)+(1+j*1)]
                    value, = struct.unpack('B', dataDecode)
                    data_decoded.append(value * 3.3/255)
                data_buffer.append(data_decoded)
            else:
                data_decoded = [1,1,1,1,1,1,1,1]            
        imu_decoded = list()
        if len(data) == 212:
            for i in range(12):
                imuDecode = data[160+i*4 : 160+4+i*4]
                value_imu, = struct.unpack('f', imuDecode)            
                imu_decoded.append(value_imu)
        else:
            imu_decoded = [1,1,1,1,1,1,1,1,1,1,1,1]  
        battery_decoded = list()
        if len(data) == 212:
            for i in range(1):
                batteryDecode = data[208+i*4 : 208+4+i*4]
                value_battery, = struct.unpack('f', batteryDecode)            
                battery_decoded.append(value_battery)
        else:
            battery_decoded = [1]            
        for sample in range(len(data_buffer)):
            data_buffer[sample].extend(imu_decoded + battery_decoded + [time.time()])#battery_decoded
        return data_buffer

    
    def sample_for_time(self, sampletime=10, gestures=None, gesture_name = None):  

        window_length = self.parameters["recording_window_length"]
        sampling_frequency = self.plots["sampling_frequency"]
        self.plots["last_plot_time"] = time.time()
        self.plots["gesture_name"] = gesture_name
        self.initialize_plot_properties(clear_axes=1)
        plot_history_time = self.plots["plot_history_time"]
        data_storage = list()
        st = time.time()
        t = 0
        self.start_transmission()
        while t < sampletime:
            a = list()
            while len(a) < window_length and t < sampletime:
                if self.PREDICTING:
                    new_samples = self.get_sample()
                    a.extend(new_samples)
                    self.plots["plotting_data"] = np.vstack([self.plots["plotting_data"], new_samples])
                    self.running_plot(testing_status = 0, predicted_result = 0)
                    t = time.time() - st
                else:
                    print("Exiting...")
                    break
            data_storage.extend(a)
            total_length = len(self.plots["plotting_data"])
            self.plots["plotting_data"] = self.plots["plotting_data"][total_length - (plot_history_time*sampling_frequency + (int)(sampling_frequency/2)): (total_length - 1),:]
        if len(a) < window_length:    
            data_storage.extend(a)
        self.stop_transmission()
        print("Length of data is {} samples. Samplerate over {} seconds is {} hz.".format(len(data_storage), round(t, 3), round(len(data_storage)/t)))
        return data_storage
    
    def sample_gestures(self, sampletime=10, gestures=None):        
        if type(gestures) != list:
            print("Please input gestures as list.")
            return
        elif type(gestures) == list:
            for gesture in gestures:
                print("Recording: {}".format(gesture))
                temp = self.sample_for_time(sampletime=sampletime,gestures=gestures, gesture_name = gesture)
                if gesture in self.data_struct["raw"]:
                    self.data_struct["raw"][gesture].extend(temp)
                elif not gesture in self.data_struct["raw"]:
                    self.data_struct["raw"][gesture] = temp
            print("Gestures recorded: {}".format(gestures))
            return self.data_struct["raw"]


    ## plot functions
    def initialize_plot_properties(self, clear_axes=None):
        
        if (clear_axes == 0):
            plt.close('all')
            fig, axs_plot = plt.subplots(3, 3, figsize=(14,6)) ## number and size of subplots can be changed from here 
            fig.tight_layout(pad=3.0)
            time.sleep(1)
            self.plots["plotting_data"] = self.plots["default_plotting_data"]
        else:
            axs_plot = self.plots["plot_axes"]
            for i in range(3):
                for j in range(3): 
                    axs_plot[i,j].clear()
            self.plots["last_plot_time"] = time.time()
            self.plots["plotting_data"] = self.plots["default_plotting_data"]

        self.plots["plot_axes"] = axs_plot
        

    def running_plot(self, testing_status = 0, predicted_result = 0): 
        
        gesture_name = self.plots["gesture_name"]
        last_plot_time = self.plots["last_plot_time"]
        axs_plot = self.plots["plot_axes"]
        gestures = self.plots["gestures"]
        indices_2_plot = self.plots["indices_2_plot"]
        data_info = self.plots["data_info"]
        data_y_limit = self.plots["data_y_limit"]
        update_plot_time  = self.plots["update_plot_time"]
        plot_history_time = self.plots["plot_history_time"]
        total_length = len(self.plots["plotting_data"])
        sampling_frequency = self.plots["sampling_frequency"]
        sample_num_array = np.arange(1, total_length+1, 1)
        data_np = np.array(self.plots["plotting_data"])
        if (time.time() - last_plot_time) > update_plot_time:
             
             last_plot_time = time.time()
             loop = 0
             for i in range(3):
               for j in range(3):
                  
                  if i==2 and j==1:
                        axs_plot[i,j].clear()
                        if(testing_status == 1):
                            if(predicted_result > -1):
                                x = gestures[predicted_result-1]
                            else:
                                x = gestures[0]
                            axs_plot[i, j].set_title('Predicted gesture')
                        else:
                            x = gesture_name
                            axs_plot[i, j].set_title('Gesture to hold')
                        
                        axs_plot[i, j].axis('off') 
                        y = '.JPG'
                        image_name =  x + y
                        cv2_img = cv2.imread(os.path.join('images', image_name))
                        mod_cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
                        axs_plot[i, j].imshow(mod_cv2_img)
                  elif i == 1 and j == 2:
                        axs_plot[i,j].axis('off')    
                  elif i<2:
                        axs_plot[i,j].clear()
                        if(total_length < plot_history_time*sampling_frequency):
                            samples_selected = sample_num_array
                            data_np_fsr = data_np[:,indices_2_plot[loop]]
                        else:
                            samples_selected = sample_num_array[total_length - plot_history_time*sampling_frequency: (total_length - 1)]
                            data_np_fsr = data_np[total_length - plot_history_time*sampling_frequency: (total_length - 1),indices_2_plot[loop]]
                        axs_plot[i, j].plot(samples_selected, data_np_fsr, linewidth=1)
                        axs_plot[i, j].set_ylim(data_y_limit[loop])
                        axs_plot[i, j].set_xlim([total_length-(plot_history_time*sampling_frequency), total_length+(update_plot_time*sampling_frequency)])
                        axs_plot[i, j].grid(color='w', linestyle='--', linewidth=0.5)
                        axs_plot[i, j].set_title(data_info[loop])
                        axs_plot[i, j].get_xaxis().set_visible(False)
                  else:
                        axs_plot[i,j].axis('off')
                        
                  loop = loop + 1
            
             plt.pause(0.001)
            
        self.plots["last_plot_time"] = last_plot_time     

       

if __name__ == '__main__':
    """
    Run example below and follow printed output to try out SDK.
    If you want to create your own data structure:
    - Armband.sample_gestures() returns raw recorded input for defined gestures.

    If you want to sample for time:
    - Armband.sample_for_time(sampletime=10) returns raw data for ten seconds. Change 'sampletime'.

    If you want to control sampleflow:
    - Armband.start_transmission(), starts data flow.
    - Armband.get_sample() returns one sample as list of 20*21.
        - One sample is send every 20 ms.
        - Loop this for however long you want.
    - Armband.stop_transmission(), ends datastream
    """
    #  Main_file
    #- Define gestures to record. Must be list. -#
    Gestures = ["open", "close"] 

    #- Create instance -#
    s = Armband() 

    s.plots["gestures"] = Gestures

    s.initialize_plot_properties(clear_axes=0)

    #- Connect to armband. Remember to pair armband with pc bluetooth. -#
    s.connect()

    #- Calibrate armband. This is instant. Make a closed fist. -#
    s.calibrate()

    #- Record defined gestures. Sampletime for each gesture is by default 10 seconds. -#
    s.sample_gestures(sampletime=50, gestures=Gestures) 

    s.close_connection()
    
    #- To Dataframe and csv -#
    dfs = dict()
    for gesture, data in s.data_struct["raw"].items():    
        dfs[gesture] = pd.DataFrame(data, columns=s.headers["full"])
        dfs[gesture].to_csv(dt.datetime.now().strftime("%m_%d_%Y_%H%M%S_") + str(gesture) + ".csv", index=False)