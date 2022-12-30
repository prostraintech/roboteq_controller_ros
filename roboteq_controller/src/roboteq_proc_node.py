#! /usr/bin/env python
import rospy
from roboteq_controller.msg import channel_values, DigitalInput, DigitalOutput, FaultFlag, RuntimeStatusFlag
from std_msgs.msg import Float32

class RoboteqProc(object):
    def __init__(self):
        self.proc_digital_input_pub = rospy.Publisher('proc/digital_input', DigitalInput, queue_size=10)
        self.proc_digital_output_pub = rospy.Publisher('proc/digital_output', DigitalOutput, queue_size=10)
        self.proc_fault_flag_pub = rospy.Publisher('proc/fault_flag', FaultFlag, queue_size=10)
        self.proc_runtime_status_flag_pub = rospy.Publisher('proc/runtime_status_flag', RuntimeStatusFlag, queue_size=10)
        self.battery_level_pub = rospy.Publisher('proc/battery_level', Float32, queue_size=10)
        rospy.Subscriber('dig_in', channel_values, self.dig_in_sub_callback)
        rospy.Subscriber('dig_out', channel_values, self.dig_out_sub_callback)
        rospy.Subscriber('fault_flag', channel_values, self.fault_flag_sub_callback)
        rospy.Subscriber('runtime_status_flag', channel_values, self.runtime_status_flag_sub_callback)
        rospy.Subscriber('volts', channel_values, self.volts_sub_callback)

    def D2B(self, n):
        return str(bin(n)[2:].zfill(16))

    def map_volt(self, x, in_min, in_max, out_min, out_max):
        return (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min

    def volts_sub_callback(self, sub_msg):
        volt = float(sub_msg.value[1]) / 10.0
        # convert roboteq volt to actual volt
        actual_volt = (1.06 * volt) - 1.18
        # map actual volt to battery level
        battery_level = self.map_volt(actual_volt, 22.0, 28.0, 0.0, 100.0)
        if battery_level > 100.0:
            battery_level = 100.0
        elif battery_level < 0.0:
            battery_level = 0.0
        self.battery_level_pub.publish(Float32(data=round(battery_level,2)))

    def dig_in_sub_callback(self, sub_msg):
        dig_in = self.D2B(sub_msg.value[0])
        dig_in = dig_in[::-1]
        self.proc_digital_input_pub.publish(
            DigitalInput(
                estop = bool(int(dig_in[0])),
                laser1 = bool(int(dig_in[1])),
                sto = bool(int(dig_in[2])),
                bumper = bool(int(dig_in[4])),
                laser2 = bool(int(dig_in[6])),
                laser3 = bool(int(dig_in[7]))
        ))

    def dig_out_sub_callback(self, sub_msg):
        dig_out = self.D2B(sub_msg.value[0])
        dig_out = dig_out[::-1]
        self.proc_digital_output_pub.publish(
            DigitalOutput(
                brake0 = bool(int(dig_out[0])),
                brake1 = bool(int(dig_out[1])),
                fieldset0 = bool(int(dig_out[2])),
                fieldset1 = bool(int(dig_out[3])) 
        ))

    def fault_flag_sub_callback(self, sub_msg):
        fault_flag = self.D2B(sub_msg.value[0])
        fault_flag = fault_flag[::-1]
        self.proc_fault_flag_pub.publish(
            FaultFlag(
                overheat = bool(int(fault_flag[0])),
                overvolt = bool(int(fault_flag[1])),
                undervolt = bool(int(fault_flag[2])),
                short = bool(int(fault_flag[3])),
                estop = bool(int(fault_flag[4])),
                motorsensor = bool(int(fault_flag[5])),
                mosfail = bool(int(fault_flag[6])),
                defconfig = bool(int(fault_flag[7])),
                stofault = bool(int(fault_flag[8]))
        ))

    def runtime_status_flag_sub_callback(self, sub_msg):
        runtime_status_flag0 = self.D2B(sub_msg.value[0])
        runtime_status_flag0 = runtime_status_flag0[::-1]
        runtime_status_flag1 = self.D2B(sub_msg.value[1])
        runtime_status_flag1 = runtime_status_flag1[::-1]
        self.proc_runtime_status_flag_pub.publish(
            RuntimeStatusFlag(
                amplim0 = bool(int(runtime_status_flag0[0])),
                amplim1 = bool(int(runtime_status_flag1[0])),
                stall0 = bool(int(runtime_status_flag0[1])),
                stall1 = bool(int(runtime_status_flag1[1])),
                looperror0 = bool(int(runtime_status_flag0[2])),
                looperror1 = bool(int(runtime_status_flag1[2])),
                safestop0 = bool(int(runtime_status_flag0[3])),
                safestop1 = bool(int(runtime_status_flag1[3])),
                fwdlimit0 = bool(int(runtime_status_flag0[4])),
                fwdlimit1 = bool(int(runtime_status_flag1[4])),
                revlimit0 = bool(int(runtime_status_flag0[5])),
                revlimit1 = bool(int(runtime_status_flag1[5])),
                amptrig0 = bool(int(runtime_status_flag0[6])),
                amptrig1 = bool(int(runtime_status_flag1[6])),
                fetsoff0 = bool(int(runtime_status_flag0[7])),
                fetsoff1 = bool(int(runtime_status_flag1[7])),
        ))

if __name__ == "__main__":
    
    rospy.init_node('roboteq_proc', log_level=rospy.INFO)
    rp = RoboteqProc()
    rospy.spin() # mantain the service open.