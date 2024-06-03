import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Activation
from sklearn.metrics import mean_squared_error
import pandas as pd
import numpy as np

columns_bottompressure = ['pressure%s'%i for i in range(12)]

columns_backpressure = ['backpressure%s'%i for i in range(12)]

columns_backbottompressure = columns_bottompressure+columns_backpressure

columns_bottompressure_diff = ['diff_pressure%s'%i for i in range(12)]

columns_backpressure_diff = ['diff_backpressure%s'%i for i in range(12)]

columns_backbottompressure_diff = columns_bottompressure_diff+columns_backpressure_diff

columns_odom = ['odom_linearx', 'odom_angularz']

columns_delayedodom = ['delayed%s_odom_%s'%(i,x) for i in range(1,7) for x in ['linearx','angularz']]
                   

def create_model(X):
    """
    Create a Keras Sequential model

    Parameter:
        X: numpy representation of the DataFrame that contains the input features

    Returns: the untrained Keras Sequential model
    """
    n_inputs=X.shape[1]
    dropout_rate = 0.4
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(512, input_dim=n_inputs, activation='relu'),
        tf.keras.layers.Dropout(dropout_rate),
        tf.keras.layers.Dense(256, activation='relu'),
        tf.keras.layers.Dropout(dropout_rate),
        tf.keras.layers.Dense(512, activation='relu'),
        tf.keras.layers.Dropout(dropout_rate),
        tf.keras.layers.Dense(256, activation='relu'),
        tf.keras.layers.Dropout(dropout_rate),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dropout(dropout_rate),
        tf.keras.layers.Dense(64, activation='relu'),
        tf.keras.layers.Dense(2)
        ])

    model.compile(loss=tf.keras.losses.MeanSquaredError(), optimizer=tf.keras.optimizers.RMSprop(learning_rate=0.0002), metrics=['mse'])
    return model
    
    
def load_data(filename_s1, filename_s3, delta):
    """
    Loads the csv files and returns the corresponding pd.DataFrame

    Parameters:
        filename_s1(s3): the name of the csv files that contain the dataset collected at speed level 1 (at speed level 3)
        delta: the time step in the past used to compute the differential pressure. Example: Delta = 3 corresponds to a time step of 0.3s.

    Returns: the pandas DataFrames that contain the data collected at speed level 1, and 3.
                 In addition to new input feature data: velocity in the past(up to 0.6 s) and
                 the differential pressure (computed with a delay determined by delta).
    """
    df_s1 = pd.read_csv(filename_s1,skiprows=[0])
    df_s3 = pd.read_csv(filename_s3,skiprows=[0])
    df_list = []
    for df in [df_s1,df_s3]:   
        df.columns = columns_backbottompressure+columns_odom+['imu.x', 'imu.y', 'imu.z','joy_ax1', 'joy_ax2', 'circuits']

        df = diffpressure(df, delta)
        for t in range(1,7):
	        df = delayedvelocity(df, t)
        df_list.append(pd.get_dummies(df, columns=['circuits']))    
    return df_list[0], df_list[1]
 
 
def diffpressure(df, delta):
    """
    Adds new columns with differential pressure values to the DataFrame

    Parameters:
        df: original DataFrame that contains the pressure values
        delta: the time delay used in the computation of the differential pressure

    Returns: pd.DataFrame, the DataFrame with new columns containing differential pressure values.
    """
    for col in columns_backbottompressure:
        dfc = df[col]
        diff_pressure = np.zeros(len(df))
        diff_pressure[1:delta] = np.array(dfc[1:delta])-np.array(dfc[0:delta-1])
        diff_pressure[delta:] = np.array(dfc[delta:])-np.array(dfc[:-delta])
        name = "diff_" + str(col)
        df[name] = diff_pressure
    return df


def delayedvelocity(df, t):
    """
    Adds new columns with past odometry values to the DataFrame

    Parameters:
        df: original DataFrame that contains the odometry (velocity) data
        t : number of past time steps to include

    Returns: pd.DataFrame, the DataFrame with new columns containing past velocity values.
    """
    for col in columns_odom:
        dfc = df[col]
        delayedvelocity = np.zeros(len(df))
        delayedvelocity[1:t] = dfc[:t-1]
        delayedvelocity[t:] = dfc[:-t]
        name = "delayed%s_"%t + str(col)
        df[name] = delayedvelocity
    return df


def get_scaled(df):
    """
    Rescales the values of the DataFrame to be between -1 and 1.

    Parameters:
         df: original DataFrame with the data to be rescaled

    Returns: Dataframe with scaled values
    """
    
    for cols,scale_factor in [(columns_bottompressure,255.),(columns_backpressure,255.),
                              (columns_bottompressure_diff,255.),(columns_backpressure_diff,255.),
                              (["odom_linearx", "odom_angularz"],1.3)]:
        icols = [df.columns.get_loc(c) for c in cols]
        df.iloc[:, icols] = df.iloc[:, icols].apply(lambda x: x / scale_factor)
    return df


    
