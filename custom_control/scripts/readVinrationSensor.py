#!/usr/bin/env python

import rospy
import time
import datetime
import serial
import csv
import time
import argparse

def remove_element(name, data):
    try:
        data.remove(name)
    except ValueError:
        pass
        # rospy.loginfo(f"No element: {name} in this string")


def main():
    rospy.loginfo("IMU logging start")
    rospy.init_node('IMU_logger',anonymous=True)
    now = datetime.datetime.now()
    dateAndTime = now.strftime('%d-%m-%y-%H-%M-%S')
    filecsv = open('/home/mtrn4230/catkin_ws/src/Thesis_robotic_drilling/log_data/'+dateAndTime+'.csv', 'w')
    writer = csv.writer(filecsv)    
    port = serial.Serial("/dev/ttyUSB0", 115200,timeout=1)
    
    while not port.is_open:
        continue

    initTime = time.time()
    header = ['t', 'w', 'x' ,'y', 'z', 'ax', 'ay' ,'az']
    writer.writerow(header)
    counter = 0
    while not rospy.is_shutdown():
        # Arduino gets reset everytime COM port is opened.
        line1 = str()
        line2 = str()
        try:
            line1 = port.readline().decode().strip()
            line2 = port.readline().decode().strip()
        except UnicodeDecodeError:
            rospy.loginfo("Error decoding")
        
        rospy.loginfo(line1)
        rospy.loginfo(line2)

        # Check each string, confirm they are whaat I expect
        # Take first three leters of ypr string. If it matches ypr, leave it in order, if it doesn't swap them over.

        if (line1[0:4] == 'quat'):
            rospy.loginfo("You have correct order")
            data = line1 + "\t" + line2
        else:
            rospy.loginfo ("you have reveresed order")
            data = line2 + "\t" + line1
        
        # Convert data to list of units
        listData = data.split('\t')
        remove_element('quat',listData)
        remove_element('areal',listData)
        remove_element('aworld', listData)
        remove_element('ypr',listData)
        prependedList = [counter] + listData
        rospy.loginfo(prependedList)
        writer.writerow(prependedList)
        counter += 1

main()
