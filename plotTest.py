import numpy as np
import serial
from numpy.fft import fft
from numpy.fft import fftshift
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
import threading
import os
from queue import Queue
import time
z1baudrate = 921600
z1port = '/dev/ttyACM2'  # set the correct port before run it

magicWord = bytearray(b'\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09')



frameQueue = Queue()


z1serial= open('Horizontal_front-tilt_faster-rpm_2fps_10mps','rb')


#z1serial = serial.Serial(port=z1port, baudrate=z1baudrate)
#z1serial.timeout = None
#print z1serial.is_open  # True for opened
i = 0

startIdx = 0
endIdx = 0


frameIdx = 0

data = z1serial.read(1671168)

numChirps = 32
numTxAntennas = 1
numRxAntennas = 4
numRangeBins = 64


numVirtualAntennas = numTxAntennas*numRxAntennas


radarCube = np.zeros((numRangeBins, numChirps, numVirtualAntennas), dtype=np.complex64)
rangeDoppler = np.zeros((numRangeBins,numChirps),dtype=np.complex64)

rangeBins = (np.linspace(0,63,64))*3/64
dopplerBins = np.linspace(0,31,32)


#rangeBins, dopplerBins = np.meshgrid(rangeBins,dopplerBins)





plt.figure()


while True:
    try:
        frame_bytes = bytearray(data[frameIdx*32768:(frameIdx+1)*32768])

    except Exception as e:
        print(e)


    idx = 0


    for chirp in range(0,numChirps):    #The odd chirps
        for rxAntenna in range(0,numRxAntennas):
            for rangeBin in range(0,numRangeBins):
                rePart = frame_bytes[idx+1]*256 + frame_bytes[idx]
                imPart = frame_bytes[idx+3]*256 + frame_bytes[idx+2]

                if(rePart > 32767):
                    rePart -= 65536
                if (imPart > 32767):
                    imPart -= 65536

                radarCube[rangeBin,chirp,rxAntenna] = rePart + 1j*(imPart)
                idx += 4




    z = fft(radarCube[:,:,3])
    #plt.imshow(abs(radarCube[:,:,3]))

    plt.clf()
    plt.plot(abs(z[:,:]))
    #plt.imshow(abs(z[:,:]))

    plt.pause(0.1)
    frameIdx = frameIdx + 1
    print(frameIdx)
    time.sleep(0.25)


