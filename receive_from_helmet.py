# -*- coding: utf-8 -*-
"""
Created on Mon Mar  5 19:42:38 2018

"""

import matplotlib.pyplot as plt
import numpy as np
import serial
import time
import datetime



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


    
ser = serial.Serial(
    port='COM11',\
    baudrate=921600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

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

# send stop command
print("Sending stop...")
ser.write(b'x')

# clear reading buffer
print("Clearing buffer...")
while ser.inWaiting()>0:
    ser.read(1)
print("bytes in buffer:",ser.inWaiting())

channels = np.zeros((max_points,number_channels)) #init numpy-array for channels


# send start command
print("Sending start...")
ser.write(b's')

print("Reading...")
for t in range(0,max_points): # time
    
    # at least one block in serial buffer, otherwise wait
   while ser.inWaiting()<45:
       time.sleep(0.01)
       
   test = ser.read(4) #read leading zeroes (4 bytes)
   if t == 0:
       print("First Bytes in Stream:", test)

   for i in range(0,number_channels): #read 13 channels (3 bytes each)
       channels[t,i] = from24bitIntegerTwos(ser.read(3)) # read first channel (3 bytes)

   ser.read(2) #read trailing zeroes (2 bytes)
      
#print(channels)
   
# send stop command
print("Sending stop...")
ser.write(b'x') 
time.sleep(0.5)

# clear reading buffer completely again
# otherwise the helmet does not finish sending
# and cannot react to new commands
print("Clearing buffer...")
while ser.inWaiting()>0:
    ser.read(1)
print("bytes in buffer:",ser.inWaiting())

plt.rcParams["figure.figsize"] = [15,5]

# x-axis
x = np.linspace(0,(max_points-1)*0.002,max_points)

# datasheet ADC ads1258: 1.0666*Vref = 0x7FFFFF = Dez 8388607 = (2^23)-1, Vref = 4.096V
channels = 1.0666*4.096*1000.0*channels/(2**23-1) #convert to mV
# amplifier in front of ADC do 100x
# electrode 10x
channels = channels / 1000.0
# 7864320
# 8388603

now = datetime.datetime.now()
filename= "./%d%02d%02d-%02d%02d%02d.csv" % (now.year, now.month, now.day, now.hour, now.minute, now.second)
fout = open(filename, 'w') #open output file

#write header
fout.write("time_s, ch1_mV, ch2_mV, ch3_mV, ch4_mV, ch5_mV, ch6_mV, ch7_mV\n")

for t in range(0,max_points):
    fout.write("%e, %e, %e, %e, %e, %e, %e, %e\n" % ((t*0.002), channels[t,0], channels[t,1], channels[t,2], channels[t,3], channels[t,4], channels[t,5], channels[t,6]))
    #print("%e, %e, %e, %e, %e, %e, %e\n" % ((t*0.002), channels[t,0],channels[t,1],channels[t,2],channels[t,3],channels[t,4],channels[t,5],channels[t,6]))
 
fout.close()


plt.plot(x,channels)
plt.show()


for i in range(0,7):  
    print("Channel:",i+1)
    plt.plot(x,channels[:,i])
    plt.show()
    print("\n\n")


FFT = np.zeros((int(max_points/2+1),number_channels)) #init numpy-array for channels
# x-axis
x_fft = np.linspace(0,250,int(max_points/2+1))

for i in range(0,7):
    FFT[:,i] = np.fft.rfft(channels[:,i])
    FFT[:,i] = abs(FFT[:,i])
    
    print("FFT:",i+1)
    plt.plot(x_fft,FFT[:,i])
    plt.show()
    print("\n\n")
    
    
filename= "./%d%02d%02d-%02d%02d%02d_FFT.txt" % (now.year, now.month, now.day, now.hour, now.minute, now.second)
fout = open(filename, 'w') #open output file

#write header
fout.write("freq fft1 fft2 fft3 fft4 fft5 fft6 fft7\n")

for f in range(0,int(max_points/2+1)):
    fout.write("%e %e %e %e %e %e %e %e\n" % ((f*250.0/int(max_points/2)), FFT[f,0], FFT[f,1], FFT[f,2], FFT[f,3], FFT[f,4], FFT[f,5], FFT[f,6]))
    #print("%e %e %e %e %e %e %e\n" % ((t*0.002), channels[t,0],channels[t,1],channels[t,2],channels[t,3],channels[t,4],channels[t,5],channels[t,6]))
 
fout.close()

ser.close()




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
