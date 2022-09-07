import csv
import sys
import argparse
from os.path import exists
mode = "BOTH"

parser = argparse.ArgumentParser(description='Find PID values')
parser.add_argument('filename', metavar='filename', type=str, nargs=1, help='File that will be parsed')
parser.add_argument('mode', metavar='mode', choices = ["ZN", "CC", "BOTH"], default = "BOTH", nargs=1, help='Type of pid tuning method that will be used')
args = parser.parse_args()

if (len(sys.argv) < 2):
    print("Please input filename")
    sys.exit()
else: 
    filename = sys.argv[1]
if (len(sys.argv) > 2):
    mode = sys.argv[2]
    mode = mode.upper()

if not exists(filename):
    print("Please input a valid file")
    sys.exit()

if (mode != "BOTH" and mode != "ZN" and mode != "CC"):
    print("Please pass in a valid method:\n ZN for Ziegler Nichol's Open Loop Method \n CC for the Cohen-Coon Open Loop Method \n BOTH for both of them")
    sys.exit()

c = 2
distance = 0.01
x1 = 0 #
x2 = -1
middle_point = -1
x3 = -1
t = -1
l = -1
max_slope = 0


def zn_open(): #Ziegler Nichols Open Loop Method
    gain_kc = (1.2/k)*t/l
    reset_time = 2 * l
    derivative_time = 0.5 * l

    print("ZN gain_kc, reset_time, derivative_time: " + str(gain_kc) + ", " + str(reset_time) + ", " + str(derivative_time))

def cc_open(): #Cohen-Coon Open Loop Method
    gain_kc = 1.35/(k*(t/l+0.092))
    reset_time = 2.5*l*(t+0.185*l)/(t+0.611*l)
    derivative_time = 0.37*l*(t/(t+0.185*l))

    print("CC gain_kc, reset_time, derivative_time: " + str(gain_kc) + ", " + str(reset_time) + ", " + str(derivative_time))



with open(filename, 'r') as csvfile:
    datareader = csv.reader(csvfile)
    datareader_list = list(datareader)[1:] #convert csv to list, get rid of headers at the top
    setpoint = float(datareader_list[len(datareader_list)-1][2]) #target value we're going towards (setpoint of the final row)
    for i in range(len(datareader_list)):
        if i>0: #ignore first point because no points behind it
            if (float(datareader_list[i][2]) - float(datareader_list[i-1][2]))/(float(datareader_list[i][0]) - float(datareader_list[i-1][0]))>c: #If there's a sudden spike in the setpoint value, we mark this point's x value as x1
                x1=float(datareader_list[i][0])
            if abs(float(datareader_list[i][1])-setpoint) < distance and x3 < 0: #If a point's y value is reeaaaally close to the setpoint, we set its x value as x3
                x3=float(datareader_list[i][0])
            if i < len(datareader_list) - 1: #ignore last point no points after it
                 if (float(datareader_list[i+1][1]) - float(datareader_list[i-1][1]))/(float(datareader_list[i+1][0]) - float(datareader_list[i-1][0])) > max_slope: #calculate max slope by doing slope formula on adjacent points
                    max_slope = (float(datareader_list[i+1][1]) - float(datareader_list[i-1][1]))/(float(datareader_list[i+1][0]) - float(datareader_list[i-1][0]))
                    middle_point = datareader_list[i] #the point with the max slope
           
    x2 = (-float(middle_point[1])/max_slope) + float(middle_point[0]) #calculate the x value of the point where the tangent line that goes through middle_point intersects the y axis
    print("x1, x2, x3: " + str(x1) + ", " + str(x2) + ", " + str(x3))
    t = x3-x2
    l = x2-x1
    print(" ")
    print("t, l: " + str(t) + ", " + str(l))
    k= setpoint - float(datareader_list[0][1])
    print(" ")

    if (mode == "CC"): #parse command line inputs to display the appropriate tuning method
        cc_open()
    elif (mode == "ZN"):
        zn_open()
    else:
        cc_open()
        zn_open()




        

