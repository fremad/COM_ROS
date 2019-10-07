#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Empty
import pandas as pd


class Recorder:

    def __init__(self):
        self.setup()
        self.GPS_data = {'secs': [], 'nsecs': [],
                         'latitude': [], 'longitude': [], 'altitude': []}

        self.imu_data = {'secs': [], 'nsecs': [], 'linear_acceleration_x': [
        ], 'linear_acceleration_y': [], 'linear_acceleration_z': [], 'angular_velocity_x': [], 'angular_velocity_y': [], 'angular_velocity_z': []}

    def GPS_callback(self, msg):
        print("GPS callback called")
        data = rospy.wait_for_message("/navsat/fix", NavSatFix)
        self.GPS_data['secs'].append(data.header.stamp.secs)
        self.GPS_data['nsecs'].append(data.header.stamp.nsecs)
        self.GPS_data['latitude'].append(data.latitude)
        self.GPS_data['longitude'].append(data.longitude)
        self.GPS_data['altitude'].append(data.altitude)

    def imu_callback(self, data):
        self.imu_data['secs'].append(data.header.stamp.secs)
        self.imu_data['nsecs'].append(data.header.stamp.nsecs)
        self.imu_data['linear_acceleration_x'].append(
            data.linear_acceleration.x)
        self.imu_data['linear_acceleration_y'].append(
            data.linear_acceleration.y)
        self.imu_data['linear_acceleration_z'].append(
            data.linear_acceleration.z)
        self.imu_data['angular_velocity_x'].append(data.angular_velocity.x)
        self.imu_data['angular_velocity_y'].append(data.angular_velocity.y)
        self.imu_data['angular_velocity_z'].append(data.angular_velocity.z)

    def write_data(self):
        GPS_df = pd.DataFrame(self.GPS_data)
        print(GPS_df)
        IMU_df = pd.DataFrame(self.imu_data)
        export_csv = IMU_df.to_csv('~/Code/imu.csv', header=True)
        export_csv = GPS_df.to_csv('~/Code/gps.csv', header=True)

        # Write to csv

    def setup(self):
        rospy.Subscriber("/dest/reached", Empty, self.GPS_callback)
        rospy.Subscriber("imu/data", Imu, self.imu_callback)


if __name__ == '__main__':
    try:
        rospy.init_node('controllnode', anonymous=True)
        rospy.loginfo("Started")
        recorder = Recorder()
        rate = 50
        r = rospy.Rate(rate)

        record_time = 45.
        counter = 0.
        while not rospy.is_shutdown() and (1.0/rate)*counter < record_time:
            r.sleep()
            counter += 1

        recorder.write_data()


        rospy.wait_for_message("/navsat/fix", NavSatFix)
    except rospy.ROSInterruptException:
        pass
