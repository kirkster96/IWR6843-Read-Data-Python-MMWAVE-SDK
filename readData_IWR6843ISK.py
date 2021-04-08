import serial
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import matplotlib.pyplot as plt

import sys
np.set_printoptions(threshold=sys.maxsize)

# Change the configuration file name
configFileName = 'xwr68xxconfig.cfg'

CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0;


# ------------------------------------------------------------------

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    #CLIport = serial.Serial('/dev/ttyACM0', 115200)
    #Dataport = serial.Serial('/dev/ttyACM1', 921600)
    
    # Windows
    CLIport = serial.Serial('COM6', 115200)
    Dataport = serial.Serial('COM5', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# ------------------------------------------------------------------

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 3
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1;
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
                
            digOutSampleRate = int(splitWords[11]);
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1]);
            chirpEndIdx = int(splitWords[2]);
            numLoops = int(splitWords[3]);
            numFrames = int(splitWords[4]);
            framePeriodicity = int(splitWords[5]);

            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters
   
# ------------------------------------------------------------------

# Funtion to read and parse the incoming data
def readAndParseData14xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength
    
    # Constants
    OBJ_STRUCT_SIZE_BYTES = 12;
    BYTE_VEC_ACC_MAX_SIZE = 2**15;
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
    maxBufferSize = 2**15;
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)
               
        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    #print(f"magicOK = {magicOK}")

    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]
        
        # Initialize the pointer index
        idX = 0
        
        # Read the header
        magicNumber = byteBuffer[idX:idX+8]
        idX += 8
        version = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        platform = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        idX += 4
        #print(f"magicNumber = {magicNumber} \t version = {version} \t totalPacketLen = {totalPacketLen} \t platform = {platform} \t frameNumber = {frameNumber} ")
        #print(f"timeCpuCycles = {timeCpuCycles} \t\t numDetectedObj = {numDetectedObj} \t numTLVs = {numTLVs} \t\t idX = {idX}")

        # UNCOMMENT IN CASE OF SDK 2
        #subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        
        
        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Check the header of the TLV message
            #print(f"byteBuffer[idX:idX+4] = {byteBuffer[idX:idX+4]}, word = {word}")
            tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            #print(f"tlv_type = {tlv_type} \t MMWDEMO_UART_MSG_DETECTED_POINTS = {MMWDEMO_UART_MSG_DETECTED_POINTS}")
            # Read the data depending on the TLV message
            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
                            
                # word array to convert 4 bytes to a 16 bit number
                word = [1, 2**8]
                tlv_numObj = np.matmul(byteBuffer[idX:idX+2],word)
                idX += 2
                tlv_xyzQFormat = 2**np.matmul(byteBuffer[idX:idX+2],word)
                idX += 2
                
                # Initialize the arrays
                rangeIdx = np.zeros(numDetectedObj,dtype = 'int16')
                dopplerIdx = np.zeros(numDetectedObj,dtype = 'int16')
                peakVal = np.zeros(numDetectedObj,dtype = 'int16')
                x = np.zeros(numDetectedObj,dtype = 'int16')
                y = np.zeros(numDetectedObj,dtype = 'int16')
                z = np.zeros(numDetectedObj,dtype = 'int16')
                #print(f"tlv_numObj = {tlv_numObj}")
                for objectNum in range(numDetectedObj):
                    
                    # Read the data for each object
                    rangeIdx[objectNum] =  np.matmul(byteBuffer[idX:idX+2],word)                    
                    idX += 2
                    dopplerIdx[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2                    
                    peakVal[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2                    
                    x[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2
                    y[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2
                    z[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2
                    #print(f"rangeIdx[{objectNum}] = {rangeIdx[objectNum]} \t dopplerIdx[{objectNum}] = {dopplerIdx[objectNum]} \t peakVal[{objectNum}] = {peakVal[objectNum]} \t x[{objectNum}] = {x[objectNum]} \t y[{objectNum}] = {y[objectNum]} \t z[{objectNum}] = {z[objectNum]} \t")

                # Make the necessary corrections and calculate the rest of the data
                rangeVal = rangeIdx * configParameters["rangeIdxToMeters"]
                dopplerIdx[dopplerIdx > (configParameters["numDopplerBins"]/2 - 1)] = dopplerIdx[dopplerIdx > (configParameters["numDopplerBins"]/2 - 1)] - 65535
                dopplerVal = dopplerIdx * configParameters["dopplerResolutionMps"]
                #x[x > 32767] = x[x > 32767] - 65536
                #y[y > 32767] = y[y > 32767] - 65536
                #z[z > 32767] = z[z > 32767] - 65536
                #print(f"tlv_xyzQFormat = {tlv_xyzQFormat} \t x = {x}")
                #x = x / tlv_xyzQFormat
                #y = y / tlv_xyzQFormat
                #z = z / tlv_xyzQFormat
                
                # Store the data in the detObj dictionary
                detObj = {"numObj": objectNum, "rangeIdx": rangeIdx, "range": rangeVal, "dopplerIdx": dopplerIdx, \
                          "doppler": dopplerVal, "peakVal": peakVal, "x": x, "y": y, "z": z}
                
                dataOK = 1             
        
  
        # Remove already processed data
        if idX > 0 and byteBufferLength > idX:
            shiftSize = totalPacketLen
               
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
            
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                

    return dataOK, frameNumber, detObj

# ------------------------------------------------------------------

# Funtion to update the data and display in the plot
def update():
     
    dataOk = 0
    global detObj
    x = []
    y = []
      
    # Read and parse the received data
    dataOk, frameNumber, detObj = readAndParseData14xx(Dataport, configParameters)
    #print(f"dataOK = {dataOk}")
    if dataOk and len(detObj["x"]) > 0:
        #print(detObj)
        x = -detObj["x"]
        y = detObj["y"]
        plt.scatter(x,y)
        plt.pause(0.05)
        #s.setData(x,y)
        #QtGui.QApplication.processEvents()
    
    return dataOk


# -------------------------    MAIN   -----------------------------------------  

# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)

# START QtAPPfor the plot
"""
app = QtGui.QApplication([])

# Set the plot 
pg.setConfigOption('background','w')
win = pg.GraphicsWindow(title="2D scatter plot")
p = win.addPlot()
p.setXRange(-0.5,0.5)
p.setYRange(0,1.5)
p.setLabel('left',text = 'Y position (m)')
p.setLabel('bottom', text= 'X position (m)')
s = p.plot([],[],pen=None,symbol='o')

win.show()
app.exec_()
"""
#plt.axis([0, 10, 0, 1])
#plt.show()
    
   
# Main loop 
detObj = {}  
frameData = {}    
currentIndex = 0
for i in range(20):
    try:
        # Update the data and check if the data is okay
        dataOk = update()


        if dataOk:
            # Store the current frame into frameData
            frameData[currentIndex] = detObj
            currentIndex += 1
        
        time.sleep(0.033) # Sampling frequency of 30 Hz
        
    # Stop the program and close everything if Ctrl + c is pressed
    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())
        CLIport.close()
        Dataport.close()
        #win.close()
        break
        
    





