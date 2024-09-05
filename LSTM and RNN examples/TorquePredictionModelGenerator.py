import tensorflow as tf
import os
import pandas as pd
import numpy as np

csv_path = 'TorqueData_noFriction_NL_NG.xlsx'

torqueData = pd.read_excel(csv_path)

torque = torqueData['Torque']

def torque_to_X_y(torqueData, window_size = 5):
    torque_as_np = torqueData.to_numpy()
    X = []
    y = []
    for i in range(len(torque_as_np)-window_size):
        row = [[a] for a in torque_as_np[i:i+5]]
        X.append(row)
        label = torque_as_np[i+5]
        y.append(label)
    return np.array(X),np.array(y)

WINDOW_SIZE = 5
X,y = torque_to_X_y(torque,WINDOW_SIZE)

X_train, y_train = X[:1300], y[:1300]
X_val, y_val = X[1300:1600], y[1300:1600]
X_test, y_test = X[1600:2300], y[1600:2300]

