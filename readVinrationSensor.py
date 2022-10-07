from multiprocessing.sharedctypes import Value
import time
import datetime
import serial
import csv
import time
import argparse


def main(args):
    print(args)
    recordTime = 0
    if args.t is None:
        print("Default time: 10s")
        recordTime = 10
    recordTime = args.t
    now = datetime.datetime.now()
    dateAndTime = now.strftime('%d-%m-%y-%H-%M-%S')
    filecsv = open('log_data/'+dateAndTime+'.csv', 'w')
    writer = csv.writer(filecsv)    
    port = serial.Serial(args.pos_arg, 115200,timeout=1)
    
    while not port.is_open:
        continue

    initTime = time.time()
    header = ['y', 'p' ,'r', 'ax', 'ay' ,'az']
    writer.writerow(header)
    while recordTime - (time.time()-initTime) > 0:
        # Arduino gets reset everytime COM port is opened.
        # ypr is always the first line, accel is always the second
        ypr = port.readline().decode().strip()
        accel = port.readline().decode().strip()
        # Check each string, confirm they are whaat I expect
        # Take first three leters of ypr string. If it matches ypr, leave it in order, if it doesn't swap them over.

        if (ypr[0:3] == 'ypr'):
            print("You have correct order")
            data = ypr + "\t" + accel
        else:
            print ("you have reveresed order")
            data = accel + "\t" + ypr
        # Convert data to list of units
        listData = data.split('\t')
        print(listData)
        
        try:
            listData.remove('ypr')
        except ValueError:
            print("No element: ypr is this string")

        try:    
            listData.remove('areal')
        except ValueError:
            print("There is no element \"areal\"")

        try:
            listData.remove('aworld')
        except ValueError:
            print("There is no element: \"aworld\"")
            
        print(listData)
        writer.writerow(listData)

parser = argparse.ArgumentParser()
parser.add_argument('pos_arg', type=str, help='Which com port do you want to connect to?')
parser.add_argument('-t', type=int, help='How long do you want to record for (seconds)?')
args=parser.parse_args()
main(args)
