#!/usr/bin/env python
import rospy
from chassis_msgs.msg import Chassis
from acu_new_ioniqw_msgs.msg import acu_new_ioniqw
from inertial_labs_msgs.msg import ilgps_RTtype
import socket
import pickle

#host = '10.10.0.10'
host = '1.249.212.151'
port = 5050
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class vehicle_info:

    def __init__(self):

        rospy.Subscriber("/chassis", Chassis, self.chassis_callback)
        rospy.Subscriber("/acu_new_ioniqw", acu_new_ioniqw, self.acu_callback)
        rospy.Subscriber("/ilgps_utm", ilgps_RTtype, self.gps_cb)
        self.sas_angle = 0.0 # deg
        self.Vx = 0.0 # kph
        self.MDPS_Module_Stat = False
        self.ACC_Module_Stat = False
        self.x_d = 0.0
        self.y_d = 0.0

    def chassis_callback(self, data):
        self.Vx = round(data.whl_spd_fl,1)
        self.sas_angle = round(data.sas_angle,1)
    def acu_callback(self,data):
        self.MDPS_Module_Stat = data.MDPS_Module_Stat
        self.ACC_Module_Stat = data.ACC_Module_Stat
    def udp_client(self,*argv):
        d = {"Vx":self.Vx, "sas_angle":self.sas_angle, "MDPS_Module_Stat":self.MDPS_Module_Stat, "ACC_Module_Stat":self.ACC_Module_Stat, "RTK" : [self.x_d, self.y_d]}
        sock.sendto(pickle.dumps3(d),(host,port))
        print(len(pickle.dumps(d)))
    def gps_cb(self, data):
        self.x_d, self.y_d = data.X_RT, data.Y_RT

def main():

    vi = vehicle_info()
    rospy.init_node('listener', anonymous=False)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        vi.udp_client(host,port)
        print('spd',vi.Vx,'steer',vi.sas_angle,'MDPS',vi.MDPS_Module_Stat,'ACC',vi.ACC_Module_Stat, "RTK", vi.x_d, vi.y_d)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
        main()
