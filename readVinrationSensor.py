import time
import datetime
import serial
import csv
import time
import argparse

def remove_element(name : str, data : list):
    try:
        data.remove(name)
    except ValueError:
        print(f"No element: {name} in this string")


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
    header = ['w', 'x' ,'y', 'z', 'ax', 'ay' ,'az']
    writer.writerow(header)
    while recordTime - (time.time()-initTime) > 0:
        # Arduino gets reset everytime COM port is opened.
        # ypr is always the first line, accel is always the second
        line1 = str()
        line2 = str()
        try:
            line1 = port.readline().decode().strip()
            line2 = port.readline().decode().strip()
        except UnicodeDecodeError:
            print("Error decoding")
        
        print(line1)
        print(line2)

        # Check each string, confirm they are whaat I expect
        # Take first three leters of ypr string. If it matches ypr, leave it in order, if it doesn't swap them over.

        if (line1[0:4] == 'quat'):
            print("You have correct order")
            data = line1 + "\t" + line2
        else:
            print ("you have reveresed order")
            data = line2 + "\t" + line1
        # Convert data to list of units
        listData = data.split('\t')
        print(listData)


        remove_element('quat',listData)
        remove_element('areal',listData)
        remove_element('aworld', listData)
        remove_element('ypr',listData)
        
        print(listData)
        writer.writerow(listData)

parser = argparse.ArgumentParser()
parser.add_argument('pos_arg', type=str, help='Which com port do you want to connect to?')
parser.add_argument('-t', type=int, help='How long do you want to record for (seconds)?')
args=parser.parse_args()
main(args)
