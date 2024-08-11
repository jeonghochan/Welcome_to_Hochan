#!/usr/bin/env python
import rospy
import os
import cantools
import time
import datetime
from can_msgs.msg import Frame
from chassis_msgs.msg import Chassis
from std_msgs.msg import String, Bool
from acu_new_ioniqw_msgs.msg import acu_new_ioniqw, acu_new_ioniqw_TX
from vdcl_gui.msg import gui_msg
from geometry_msgs.msg import Point

class acu_new_ioniqw_can_parser:

    def __init__(self):

        self.test_mode = True

        script_dir = os.path.dirname(__file__)
        can_db_path = os.path.join(script_dir, '../DBC/new-ioniqw/IONIQ_ACU_DBC(rev004_200827).dbc')
        chassis_can_db_path = os.path.join(script_dir, '../DBC/new-ioniqw/CAN1_Chassis_AutoBox+hazardswitch.dbc')


        self.db = cantools.database.load_file(can_db_path)
        self.chassis_db = cantools.database.load_file(chassis_can_db_path)
        self.cb_time = time.time()
        self.gui_stop = False
        self.gui_mdps = False
        self.gui_acc = False
        self.gui_automation = False
        self.ibeo_status = False
        self.velo_status = False
        self.can1_status = True
        self.can3_status = True
        self.can_status = True
        self.route_status = True
        self.v2x_status = True
        self.route_status = True

        self.MDPS_Enable_pre = False
        self.MDPS_Enable_now = False
        self.Disable_mask = False
        self.alpha = 0.0
        self.SAS_Angle = 0.0
        self.SWA_cmd_smooth = 0.0
        self.SWA_cmd_pre = 0.0
        self.trigger_license = 0
        self.isNAgear = True

        #vehicle to ROS
        rospy.Subscriber("/can/chassis/rx", Frame, self.rx_callback)
        rospy.Subscriber("/can/chassis/rx", Frame, self.sas_rx_callback)
        rospy.Subscriber("/chassis", Chassis, self.chassis_callback)
        self.rx_pub = rospy.Publisher("/acu_new_ioniqw", acu_new_ioniqw, queue_size=1)

        #ROS to vehicle
        rospy.Subscriber("/acu_new_ioniqw_TX", acu_new_ioniqw_TX, self.tx_callback)
        rospy.Subscriber("/gui_msg", gui_msg, self.gui_callback)
        rospy.Subscriber("/ibeo_status", Bool, self.ibeo_status_callback)
        rospy.Subscriber("/velodyne_status", Bool, self.velo_status_callback)
        rospy.Subscriber("/can1_status",Bool,self.can1_status_callback)
        rospy.Subscriber("/can3_status", Bool, self.can3_status_callback)
        rospy.Subscriber("/route_status", Bool, self.route_status_callback)
        rospy.Subscriber("/cam_status", Bool, self.vision_status_callback)
        rospy.Subscriber("/v2x_status", Bool, self.v2x_status_callback)
        rospy.Subscriber("/UDP_input", Point, self.key_val_callback)
        self.tx_pub = rospy.Publisher("/can/acu_new_ioniqw/tx", Frame, queue_size=1)
        self.rx_data = acu_new_ioniqw()
        self.tx_data = Frame()
        self.Ax = 0.0
        self.SWA = 0.0

    def key_val_callback(self, data):
        # (Ax : -3.0~+3.0, SWA : -450~+450)
        axis0 = data.x
        axis1 = data.y
        axis2 = data.z
        self.SWA = round(-4.5*axis0,5)
#        self.SWA = axis1
        ac, br = 100 - axis1, 100 - axis2
        self.Ax = min(round((ac - br)/200*3,5), 1.5)
        print('accel:',self.Ax,'steering:',self.SWA)


    def ibeo_status_callback(self, data):
        self.ibeo_status = data.data
    def velo_status_callback(self, data):
        self.velo_status = data.data
    def can1_status_callback(self, data):
        self.can1_status = data.data
    def can3_status_callback(self, data):
        self.can3_status = data.data
    def route_status_callback(self, data):
        self.route_status = data.data
    def v2x_status_callback(self, data):
        self.v2x_status = data.data
    def vision_status_callback(self, data):
        self.vision_status = data.data

    def rx_callback(self, data):
        try:
            signals = self.db.decode_message(data.id, data.data)
        except KeyError:
            return

        if data.id == 1808: # IONIQ_ACU_TX (0x710)

            self.rx_data.OVR_APS = signals['OVR_APS']
            self.rx_data.OVR_BPS = signals['OVR_BPS']

            self.MDPS_Enable_pre = self.MDPS_Enable_now
            self.rx_data.MDPS_Module_Stat = signals['MDPS_Module_Stat']
            self.MDPS_Enable_now = self.rx_data.MDPS_Module_Stat

            self.rx_data.ACC_Module_Stat = signals['ACC_Module_Stat']
            self.rx_data.MDPS_Enable_Report = signals['MDPS_Enable_Report']
            self.rx_data.EMR_STOP_Enable_Report = signals['EMR_STOP_Enable_Report']
            self.rx_data.ACC_Enable_Report = signals['ACC_Enable_Report']
            self.rx_data.SWA_Report = signals['SWA_cmd_Report']
            self.rx_data.Ax_cmd_Report = signals['Ax_cmd_Report']
            self.rx_data.OVR_STR = signals['OVR_STR']
            self.rx_data.Turn_Signal_Left_Report = signals['Turn_Signal_Left_Report']
            self.rx_data.Turn_Signal_Right_Report = signals['Turn_Signal_Right_Report']
            self.rx_data.ACU_AliveCounter = signals['ACU_AliveCounter']
            self.rx_data.MDPS_OVR_Prevention_Report = signals['MDPS_OVR_Prevention_Report']

    def sas_rx_callback(self, data):
        try:
            chassis_signals = self.chassis_db.decode_message(data.id, data.data)
        except KeyError:
            return

        if data.id == 688:
            self.SAS_Angle = chassis_signals['SAS_Angle']

    def chassis_callback(self, data):
        self.trigger_license = data.cf_clu_cruiseswstate
        self.isNAgear = (data.gear == 4) or (data.gear == 8)

    def tx_callback(self, data):
        try:
            today = datetime.datetime.today()
            year = today.year
            month = today.month
            day = today.day
            hour = today.hour
            minute = today.minute
            second = today.second
            microsecond = today.microsecond
		
            command_CAN = {}
            command_CAN['Left_Turn_Signal'] = data.Left_Turn_Signal
            command_CAN['Right_Turn_Signal'] = data.Right_Turn_Signal
            command_CAN['MDPS_Enable'] = (not self.Disable_mask) and (not self.isNAgear) and (self.gui_mdps and self.gui_automation and self.can_status and self.can3_status and self.velo_status and self.route_status and self.vision_status)
            command_CAN['EMR_STOP_Enable'] = False
            command_CAN['ACC_Enable'] = (not self.Disable_mask) and (not self.isNAgear) and (self.gui_acc and self.gui_automation and self.can_status and self.can3_status and self.velo_status and self.route_status and self.vision_status)
            command_CAN['MDPS_OVR_Prevention'] = False
#            print(command_CAN)
            self.alpha = min(1, self.alpha + 0.01)
            if (self.MDPS_Enable_pre == False and self.MDPS_Enable_now == True) or (self.MDPS_Enable_now == False):
                self.alpha = 0.0

            self.SWA_cmd_smooth = self.alpha * (0.7*data.SWA_cmd + 0.3*self.SWA_cmd_pre) + (1 - self.alpha) * self.SAS_Angle
            self.SWA_cmd_pre = data.SWA_cmd
            command_CAN['SWA_cmd'] =  self.SWA_cmd_smooth
            command_CAN['Ax_cmd'] = max(-5.0, data.Ax_cmd)
            command_CAN['EMR_Ax_cmd'] = data.EMR_Ax_cmd
            can_msg_command_CAN = self.db.encode_message(342, command_CAN)
            self.tx_data.data = can_msg_command_CAN
            # IONIQ_ACU_RX (0x156)
            self.tx_data.id = 342
            self.tx_data.dlc = 8
            self.tx_data.is_error = False
            self.tx_data.is_rtr = False
            self.tx_data.is_extended = False
            self.tx_data.header.frame_id = "O"
            self.tx_data.header.stamp = rospy.Time.now()
            self.tx_pub.publish(self.tx_data)

#            print(command_CAN)
        except KeyError:
            return

    def gui_callback(self, data):
        try:
            self.gui_automation = data.Automation_on
            self.gui_mdps = data.LKAS_switch
            self.gui_acc = data.SCC_switch
            self.gui_stop = data.Stop_signal

        except KeyError:
            return

    def test_publish(self):
        try:
            today = datetime.datetime.today()
            year = today.year
            month = today.month
            day = today.day
            hour = today.hour
            minute = today.minute
            second = today.second
            microsecond = today.microsecond

            command_CAN = {}
            command_CAN['Left_Turn_Signal'] = False
            command_CAN['Right_Turn_Signal'] = False
            command_CAN['MDPS_Enable'] = True
            command_CAN['EMR_STOP_Enable'] = False
            command_CAN['ACC_Enable'] =  True
            command_CAN['MDPS_OVR_Prevention'] = False
            self.alpha = min(1, self.alpha + 0.01)
            if (self.MDPS_Enable_pre == False and self.MDPS_Enable_now == True) or (self.MDPS_Enable_now == False):
                self.alpha = 0.0

            self.SWA_cmd_smooth = self.alpha * (0.7*self.SWA + 0.3*self.SWA_cmd_pre) + (1 - self.alpha) * self.SAS_Angle
            self.SWA_cmd_pre = self.SWA
            command_CAN['SWA_cmd'] =  self.SWA_cmd_smooth
            command_CAN['Ax_cmd'] = self.Ax
            command_CAN['EMR_Ax_cmd'] = 0.0
            can_msg_command_CAN = self.db.encode_message(342, command_CAN)
            self.tx_data.data = can_msg_command_CAN
            # IONIQ_ACU_RX (0x156)
            self.tx_data.id = 342
            self.tx_data.dlc = 8
            self.tx_data.is_error = False
            self.tx_data.is_rtr = False
            self.tx_data.is_extended = False
            self.tx_data.header.frame_id = "O"
            self.tx_data.header.stamp = rospy.Time.now()
            self.tx_pub.publish(self.tx_data)

#            print(command_CAN)
        except KeyError:
            return

def main():

    rospy.init_node('acu_new_ioniqw_can_parser', anonymous=False)
    rate = rospy.Rate(50) # 50hz
    cp = acu_new_ioniqw_can_parser()

    while not rospy.is_shutdown():
        try:
            cp.rx_data.header.stamp = rospy.Time.now()
            cp.tx_data.header.stamp = rospy.Time.now()
        
#        if cp.trigger_license > 3:
#            cp.Disable_mask = False
#        if cp.can1_status is False:
#            cp.Disable_mask = True

            if cp.test_mode:
                cp.test_publish()
            #rospy.loginfo(cp.rx_data)
            cp.rx_pub.publish(cp.rx_data)
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            continue
        except rospy.ROSInterruptException:
            quit();

if __name__ == '__main__':
	main()
