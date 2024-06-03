import tensorflow as tf
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from algorithms import *

# ----- parameters ----------------
delta = 3 # time step in the past for the computation of the differential pressure
input_features = (columns_backbottompressure + columns_backbottompressure_diff + \
					columns_odom)  # choice of the set of input features
BATCHSIZE = 256
EPOCHS = 400

# define the csv files on interest
filenames_s1 = ["user1_s1.csv", "user2_s1.csv", "user3_s1.csv"] # For the files containing data acquired at speed level 1
filenames_s3 = ["user1_s3.csv", "user2_s3.csv", "user3_s3.csv"] # For the files containing data acquired at speed level 3
# ----- ---------- ----------------
 
# ----- code starts -------------------
Dir = "../data/csv_files/" 
df_s1 = [-1]*len(filenames_s1)
df_s3 = [-1]*len(filenames_s3)

# load the data as csv file and create a panda dataframe
for k in range(len(filenames_s1)):
	df_s1[k], df_s3[k] = load_data(Dir+filenames_s1[k], Dir+filenames_s3[k], delta)


df_train = pd.concat(df_s1 + df_s3, axis=0)     
df_train = get_scaled(df_train)

# define the X_train, y_train, X_train_val, y_train_val from the entire dataset     
X_train = df_train[input_features].values
y_train = df_train[['joy_ax1', 'joy_ax2']].values


# define the model
model = create_model(X_train)

log = model.fit(
            X_train,
            y_train,
            batch_size=BATCHSIZE,
            epochs=EPOCHS,
            verbose=0,
            validation_split = 0.3,
            #validation_data=(X_train_val, y_train_val),
            callbacks=[tf.keras.callbacks.EarlyStopping(patience=10)],
        )
        
model.save("Delta%s_FeaturesNumber%s.keras" %(delta, len(input_features)), overwrite=True, save_format="tf")       
