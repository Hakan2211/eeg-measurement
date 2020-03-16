# -*- coding: utf-8 -*-
"""
Created on Mon Mar  5 19:42:38 2018

"""

import matplotlib.pyplot as plt
import numpy as np
import serial
import time
import datetime


x = np.random.rand(50000,13)
def data_generator():
    for i in range(len(x[:,0])):
        yield x[i]

def from24bitIntegerTwos(three_bytes):
    value =  three_bytes[0] << 16 | three_bytes[1] << 8 | three_bytes[2]
    return twos_complement(value, 24)



def twos_complement(value, bitWidth):
    if value >= 1 << bitWidth:
        # This catches when someone tries to give a value that is out of range
        raise ValueError("Value: {} out of range of {}-bit value.".format(value, bitWidth))
    else:
        #return value - int((value << 1) & 2**bitWidth)
        return value - int((value << 1) & 1 << bitWidth)


    
# ser = serial.Serial(
#     port='COM11',\
#     baudrate=921600,\
#     parity=serial.PARITY_NONE,\
#     stopbits=serial.STOPBITS_ONE,\
#     bytesize=serial.EIGHTBITS,\
#         timeout=0)

ser = data_generator()

max_points = 4096 #8 sec
#max_points = 16384 #32 sec
#max_points = 49152 #96 sec
#max_points = 24576 #49 sec
#max_points = 2097152 # 2^21 = 69min
#max_points = 1048576 # 2^20 = 34min
#max_points = 524288 # 2^19 = 17min
#max_points = 65536 # 2^16 = 2min 10s
#max_points = 32768 # 2^15 = 66s
#max_points = 16384 # 2^14 = 33s
number_channels = 13

#time.sleep(10)

# # send stop command
# print("Sending stop...")
# ser.write(b'x')

# # clear reading buffer
# print("Clearing buffer...")
# while ser.inWaiting()>0:
#     ser.read(1)
# print("bytes in buffer:",ser.inWaiting())

channels = np.zeros((max_points,number_channels)) #init numpy-array for channels


# send start command
# print("Sending start...")
# ser.write(b's')

print("Reading...")
step = 5
window = 100
plot_window = np.zeros((window, number_channels))
plt.ion()
plt.rcParams["figure.figsize"] = [15,5]

# x-axis
x_axis = np.linspace(0,(window)*0.002,window)

now = datetime.datetime.now()
filename= "./%d%02d%02d-%02d%02d%02d.csv" % (now.year, now.month, now.day, now.hour, now.minute, now.second)
fout = open(filename, 'w') #open output file

#write header
fout.write("time_s, ch1_mV, ch2_mV, ch3_mV, ch4_mV, ch5_mV, ch6_mV, ch7_mV\n")

ax1 = plt.subplot(1,2,1)
ax2 = plt.subplot(1,2,2)
for t in range(0,max_points): # time
    
#     # at least one block in serial buffer, otherwise wait
#    while ser.inWaiting()<45:
#        time.sleep(0.01)
       
#    test = ser.read(4) #read leading zeroes (4 bytes)
#    if t == 0:
#        print("First Bytes in Stream:", test)

#    for i in range(0,number_channels): #read 13 channels (3 bytes each)
#        channels[t,i] = from24bitIntegerTwos(ser.read(3)) # read first channel (3 bytes)
    channels[t] = next(ser)
    fout.write("%e, %e, %e, %e, %e, %e, %e, %e\n" % ((t*0.002), channels[t,0], channels[t,1], channels[t,2], channels[t,3], channels[t,4], channels[t,5], channels[t,6]))
    if t % step == 0 and t > 0:
        channels[t-step+1:t+1] = 1.0666*4.096*channels[t-step+1:t+1]/(2**23-1)
        plot_window[:-step] = plot_window[step:]
        plot_window[window-step:] = channels[t-step+1:t+1]
        ax1.cla()
        ax2.cla()
        for j in range(7):

            ax1.plot(x_axis, plot_window[:,j], label = "ch: "+str(j+1))
            ax1.legend(loc = 2)

            plot_fft = np.abs(np.fft.rfft(plot_window[:,j]))

            ax2.plot(x_axis[::2], plot_fft[1:], label = "ch: "+str(j+1))
            ax2.legend(loc = 2)
        # plt.plot(plot_window[:,0])
        plt.show()
        plt.pause(0.2)

fout.close()
#    ser.read(2) #read trailing zeroes (2 bytes)
      
#print(channels)
   
# # send stop command
# print("Sending stop...")
# ser.write(b'x') 
# time.sleep(0.5)

# clear reading buffer completely again
# otherwise the helmet does not finish sending
# and cannot react to new commands
# print("Clearing buffer...")
# while ser.inWaiting()>0:
#     ser.read(1)
# print("bytes in buffer:",ser.inWaiting())

# ser.close()




# =============================================================================
# How it works:
# 
# First, we make sure that the user has passed us a value that is within the 
# range of the supplied bit range (e.g. someone gives us 0xFFFF and specifies 8 bits) 
# Another solution to that problem would be to bitwise AND (&) the value with (2**bitWidth)-1
# 
# To get the result, the value is shifted by 1 bit to the left. This moves the MSB of the value
# (the sign bit) into position to be anded with 2**bitWidth = 1 << bitWidth. When the sign bit is '0' the
# subtrahend becomes 0 and the result is value - 0. When the sign bit is '1' the subtrahend
# becomes 2**bitWidth and the result is value - 2**bitWidth
# 
# Example 1: If the parameters are value=0xFF (255d, b11111111) and bitWidth=8
# 
#     0xFF - int((0xFF << 1) & 2**8)
#     0xFF - int((0x1FE) & 0x100)
#     0xFF - int(0x100)
#     255 - 256
#     -1
# 
# Example 2: If the parameters are value=0x1F (31d, b11111) and bitWidth=6
# 
#     0x1F - int((0x1F << 1) & 2**6)
#     0x1F - int((0x3E) & 0x40)
#     0x1F - int(0x00)
#     31 - 0
#     31
# 
# Example 3: value = 0x80, bitWidth = 7
# 
# ValueError: Value: 128 out of range of 7-bit value.
# 
# Example 4: value = 0x80, bitWitdh = 8
# 
#     0x80 - int((0x80 << 1) & 2**8)
#     0x80 - int((0x100) & 0x100)
#     0x80 - int(0x100)
#     128 - 256
#     -128
# 
# Now, using what others have already posted, pass your bitstring into int(bitstring,2) and pass to the twos_complement method's value parameter.
# 
# =============================================================================
