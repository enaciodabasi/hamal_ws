#!/usr/bin/env python3

import serial
import time as t
import serial.tools.list_ports as port_list
import re
import os
import rospy
import signal
import sys
from hamal_custom_interfaces.msg import BmsStatus
PortName = None


def GetPortList():
    ports = list(port_list.comports())
    for port, desc, hwid in sorted(ports):
        #print("{}: {} [{}]".format(port, desc, hwid))
        return str(port)


def SetPortConfig():
    global PortName

    Ser = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)

    if (Ser.isOpen()):
        return Ser
    else:
        return -1
# ----------------------------------------------------------------


def GetSerialData(Ser):
    rate = rospy.Rate(1) # 200 ms -> 5Hz
    bms_msg = BmsStatus()
    
    while Ser.isOpen():
        # global VoltageList, CurrentList, TemperatureList
        VoltageList = []
        Cell_VoltageList = []
        CurrentList = []
        TemperatureList = []
        Cell_TemperatureList =[]          
        SerialData = str(Ser.readline())
        # #print(SerialData)
        if SerialData[2] == 'V':
            TempVoltage = SerialData
            VoltageList = TextSplit("V=:\\rb'", TempVoltage)
            VoltageList = list(filter(None, VoltageList))     # listenin başındaki float olmayan elemanların silinmesi        
            for i in range(0,len(VoltageList)):                
                VoltageList[i] = float(VoltageList[i]) / 100
            Cell_VoltageList = VoltageList[3:]
            for i in range(0,len(Cell_VoltageList)):                
                Cell_VoltageList[i] = float(Cell_VoltageList[i]) / 10
            # --------- ROS Messages---------
            
            bms_msg.battery_voltage = VoltageList[0] 
            bms_msg.load_voltage = VoltageList[1]
            bms_msg.charger_voltage = VoltageList [2] 
            bms_msg.cell_voltage=Cell_VoltageList
            CleanList(VoltageList)
            #print(f'Voltage List -> {VoltageList}\n')
            # rospy.loginfo('Voltage-> ' + str(VoltageList))
        elif SerialData[2] == 'A':
            TempCurrent = SerialData
            CurrentList = TextSplit("A=:\\rb'", TempCurrent)
            CurrentList = list(filter(None, CurrentList))
            for i in range(0,len(CurrentList)):                
                CurrentList[i] = float(CurrentList[i])
            # --------- ROS Messages---------
            bms_msg.current_income = CurrentList[0] / 100    # Pile gelen akım (A)
            bms_msg.capacity = CurrentList[1] / 10           # Pilin doluluk kapasitesi (Ah)
            bms_msg.capacity_max = CurrentList[2] / 10       # Pilin total kapasitesi (Ah)
            bms_msg.soc = CurrentList[3]                     # Charge percentage (%)
            bms_msg.current_charger = CurrentList[4] / 100   # Charger dan gelen akım (A)           
            bms_msg.current_load = CurrentList[5] / 100      # Sistemin Çektiği Akım (A)
            CleanList(CurrentList)
            #print(f'Current List -> {CurrentList}\n')
            # rospy.loginfo('Current-> ' + str(CurrentList))

        elif SerialData[2] == 'T':
            TempTemperature = SerialData
            TemperatureList = TextSplit("T=:\\rb'", TempTemperature)
            TemperatureList = list(filter(None, TemperatureList))
            for i in range(0,len(TemperatureList)):                
                TemperatureList[i] = float(TemperatureList[i])
            Cell_TemperatureList = TemperatureList[1:]
            # --------- ROS Messages---------                        
            bms_msg.temperature_bms = TemperatureList[0]
            bms_msg.temperature_cell=Cell_TemperatureList
            CleanList(TemperatureList)
            #print(f'Temperature List -> {TemperatureList}\n')
            # rospy.loginfo('Temperature-> ' + str(TemperatureList))

        # BMS_info()
        battery_publisher.publish(bms_msg)       
        rate.sleep()
    else:
        Ser.close()
# ----------------------------------------------------------------


def WriteSerialData(Ser):
    Delay = 0.1  # Time Delay
    ReadMilis = 1000  # Data Reading Periode From BMS
    WriteList = ['# C\r', '?V \r', '?A \r', '?T \r', '# ' +
                 str(ReadMilis) + ' \r']  # BMS Initialize Command
    for i in range(0, len(WriteList)):
        Ser.write(str.encode(WriteList[i]))
        t.sleep(Delay)
    Ser.flushInput()  # Serial Buffer Input Clean
# ----------------------------------------------------------------


def TextSplit(delimiters, string, maxsplit=0):
    # BMS Data Split Using Regular Expression
    RegexPattern = '|'.join(map(re.escape, delimiters))
    return re.split(RegexPattern, string, maxsplit)
# ----------------------------------------------------------------


def CleanList(List):
    """
    Parameters
    List : List Object with ''
    ----------
    Returns
    List Object without '' token
    """
    while '' in List:
        List.remove('')

def Main():  # Main Function
    global PortName, battery_publisher
    PortName = GetPortList()
    if SetPortConfig() == -1:
        #print("Cannot Open Serial Port...")
        exit()
    else:
        rospy.init_node('bms_status_node')
        battery_publisher = rospy.Publisher('/hamal/bms_status', BmsStatus, queue_size=1, latch=True)
        SerObj = SetPortConfig()  # SerObj -> Serial Port Object
        WriteSerialData(SerObj)
        while not rospy.is_shutdown():  # While True Until Shutdown Node
            GetSerialData(SerObj)

if __name__ == "__main__":
    Main()

