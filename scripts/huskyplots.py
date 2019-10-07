import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

'''
This file is for plotting purposes only, and is therefore not further documented.
'''


def importdata():
    return pd.read_csv('~/Code/gps.csv'), pd.read_csv('~/Code/imu.csv')


def plot_lin_acc(df):
    df.plot(y=['linear_acceleration_x', 'linear_acceleration_y'],
            color=['r', 'b'], use_index=True)

    plt.title('Husky Linear acceleration')
    plt.xlabel('n iteration (50 hz)')
    plt.ylabel('IMU linear acceleration output')
    plt.show()


def plot_ang_vel(df):
    df.plot(y=['angular_velocity_x', 'angular_velocity_y'],
            color=['r', 'b'], use_index=True)

    plt.title('Husky angular velocity')
    plt.xlabel('n iteration (50 hz)')
    plt.ylabel('IMU angular velocity output')
    plt.show()


def plot_gps(gps_df):
    gps_df.plot(y=['longitude', 'latitude','altitude'],
            color=['r', 'b','g'], use_index=True)
    plt.show()

def main():
    [df_gps, df_imu] = importdata()
    plot_lin_acc(df_imu)
    plot_ang_vel(df_imu)

if __name__ == '__main__':
    main()
