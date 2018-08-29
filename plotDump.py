import numpy as np
import serial
from numpy.fft import fft
from numpy.fft import fftshift
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
import threading
import os
from queue import Queue


sport = '/dev/ttyACM2'  # set the correct port before run it
numTxAntennas = 1
numRxAntennas = 4
numRangeBins = 8
sbaudrate = 921600
numChirps = 128
readLen = numTxAntennas * numRxAntennas * numRangeBins * numChirps * 4

magicWord = bytearray(b'\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09')
file_name = raw_input("\n orientation of object \n")
frameQueue = Queue()
dump = open(file_name,'wb')
sserial = serial.Serial(port=sport, baudrate=sbaudrate)
sserial.timeout = None
print(sserial.is_open)  # True for opened
i = 0
startIdx = 0
endIdx = 0
frameIdx = 0





def readSerial():
    while(True):
        data = sserial.read(readLen*2+100)
	#print(data)
        startIdx = data.find(magicWord) + 10
        if(startIdx<readLen):
            frameQueue.put(data[startIdx:startIdx+readLen])
            dump.write(data[startIdx:startIdx+readLen])
            dump.flush()
        sserial.flushInput()
        sserial.flushOutput()

serialThread = threading.Thread(target=readSerial)
serialThread.start()

numVirtualAntennas = numTxAntennas*numRxAntennas

radarCube = np.zeros((numRangeBins, numChirps, numVirtualAntennas), dtype=np.complex64)
rangeDoppler = np.zeros((numRangeBins,numChirps),dtype=np.complex64)

rangeBins = (np.linspace(0,numRangeBins-1,numRangeBins))*3/64
dopplerBins = np.linspace(0,numChirps-1,numChirps)


#rangeBins, dopplerBins = np.meshgrid(rangeBins,dopplerBins)


#plt.ylim([0,100000])
#plt.autoscale(False)

while True:
    while frameQueue.empty() != True:
        try:
            frame_bytes = bytearray(frameQueue.get())
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
	plt.clf()
	plt.plot(abs(radarCube[:,:,3]))
	ax = plt.gca()
	ax.set_ylim([0, 1000])
        #ax.plot(abs(z[:,:]))
        #ax.imshow(abs(z[:,:]))
        plt.pause(0.01)
        frameIdx = frameIdx + 1
        print(frameIdx)



