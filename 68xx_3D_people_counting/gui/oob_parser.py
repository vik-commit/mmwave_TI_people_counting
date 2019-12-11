import struct
import sys
import serial
import binascii
import time
import numpy as np
import math


#Initialize this Class to create a UART Parser. Initialization takes one argument:
# 1: String Lab_Type - These can be:
#   a. 3D People Counting
#   b. SDK Out of Box Demo
#   c. Long Range People Detection
#   d. Indoor False Detection Mitigation
#   e. (Legacy): Overhead People Counting
#   f. (Legacy) 2D People Counting
# Default is (f). Once initialize, call connectComPorts(self, UartComPort, DataComPort) to connect to device com ports.
# Then call readAndParseUart() to read one frame of data from the device. The gui this is packaged with calls this every frame period.
# readAndParseUart() will return all radar detection and tracking information.
class uartParserSDK():
    def __init__(self,type='(Legacy) 2D People Counting'):
        self.headerLength = 52
        self.magicWord = 0x708050603040102
        self.threeD = 0
        self.ifdm = 0
        self.replay = 0
        self.SDK3xPointCloud = 0
        self.SDK3xPC = 0
        self.capon3D = 0
        self.aop = 0
        self.maxPoints = 1150
        if (type=='(Legacy): Overhead People Counting'):
            self.threeD = 1
        elif (type=='Indoor False Detection Mitigation'):
            self.ifdm = 1
        elif (type=='Replay'): # unused
            self.replay = 1
        elif (type=="SDK Out of Box Demo"):
            self.SDK3xPointCloud = 1
        elif (type=="Long Range People Detection"):
            self.SDK3xPC = 1
        elif (type=='3D People Counting'):
            self.capon3D = 1
        elif (type == 'Capon3DAOP'): #unused
            self.capon3D = 1
            self.aop = 1
        #data storage
        self.pcPolar = np.zeros((5,self.maxPoints))
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.numDetectedObj = 0
        self.targetBufPing = np.ones((10,20))*-1
        self.indexBufPing = np.zeros((1,self.maxPoints))
        self.classifierOutput = []
        self.frameNum = 0
        self.missedFrames = 0
        self.byteData = bytes(1)
        self.oldData = []
        self.indexes = []
        self.numDetectedTarget = 0
        self.fail = 0
        self.unique = []
        self.savedData = []
        self.saveNum = 0
        self.replayData = []
        self.startTimeLast = 0
        self.saveReplay = 0
        self.savefHist = 0
        self.saveBinary = 0
        self.fHistRT = np.empty((100,1), dtype=np.object)
        self.plotDimension = 0
        self.getUnique = 0

#below funtions are used for converting output of labs that do not match SDK 3.x DPIF output
    #convert 2D polar People Counting to 3D Cartesian
    def polar2Cart(self):
        self.pcBufPing = np.empty((5,self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[1,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[1,n])  #y
            self.pcBufPing[0,n] = self.pcPolar[0,n]*math.sin(self.pcPolar[1,n])  #x
        self.pcBufPing[3,:] = self.pcPolar[2,0:self.numDetectedObj] #doppler
        self.pcBufPing[4,:] = self.pcPolar[3,0:self.numDetectedObj] #snr
        self.pcBufPing[2,:self.numDetectedObj] = 0                                 #Z is zero in 2D case

    #convert 3D people counting polar to 3D cartesian
    def polar2Cart3D(self):
        self.pcBufPing = np.empty((5,self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[2,n] = self.pcPolar[0,n]*math.sin(self.pcPolar[2,n]) #z
            self.pcBufPing[0,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.sin(self.pcPolar[1,n]) #x
            self.pcBufPing[1,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.cos(self.pcPolar[1,n]) #y
        self.pcBufPing[3,:] = self.pcPolar[3,0:self.numDetectedObj] #doppler
        self.pcBufPing[4,:] = self.pcPolar[4,0:self.numDetectedObj] #snr
        #print(self.pcBufPing[:,:10])

    #decode People Counting TLV Header
    def tlvHeaderDecode(self, data):
        tlvType, tlvLength = struct.unpack('2I', data)
        return tlvType, tlvLength

    #decode People Counting Point Cloud TLV
    def parseDetectedObjects(self, data, tlvLength):
        objStruct = '4f'
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int((tlvLength)/16)
        for i in range(self.numDetectedObj):
            try:
                self.pcPolar[0,i], self.pcPolar[1,i], self.pcPolar[2,i], self.pcPolar[3,i] = struct.unpack(objStruct,data[:objSize])
                data = data[16:]
            except:
                self.numDectedObj = i
                break
        self.polar2Cart()

    #decode IFDM point Cloud TLV
    def parseDetectedObjectsIFDM(self, data, tlvLength):
        pUnitStruct = '4f'
        pUnitSize = struct.calcsize(pUnitStruct)
        pUnit = struct.unpack(pUnitStruct, data[:pUnitSize])
        data = data[pUnitSize:]
        objStruct = '2B2h'
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int((tlvLength-16)/objSize)
        #print('Parsed Points: ', self.numDetectedObj)
        for i in range(self.numDetectedObj):
            try:
                az, doppler, ran, snr = struct.unpack(objStruct, data[:objSize])
                data = data[objSize:]
                #get range, azimuth, doppler, snr
                self.pcPolar[0,i] = ran*pUnit[2]           #range
                if (az >= 128):
                    az -= 256
                self.pcPolar[1,i] = math.radians(az*pUnit[0])   #azimuth
                self.pcPolar[2,i] = doppler*pUnit[1]       #doppler
                self.pcPolar[3,i] = snr*pUnit[3]           #snr
            except:
                self.numDetectedObj = i
                break
        self.polar2Cart()

    #decode 3D People Counting Point Cloud TLV
    def parseDetectedObjects3D(self, data, tlvLength):
        objStruct = '5f'
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int(tlvLength/20)
        for i in range(self.numDetectedObj):
            try:
                self.pcPolar[0,i], self.pcPolar[1,i], self.pcPolar[2,i], self.pcPolar[3,i], self.pcPolar[4,i] = struct.unpack(objStruct,data[:objSize])
                data = data[20:]
            except:
                self.numDectedObj = i
                print('failed to get point cloud')
                break
        self.polar2Cart3D()

    #support for Capoin 3D point cloud
    #decode Capon 3D point Cloud TLV
    def parseCapon3DPolar(self, data, tlvLength):
        pUnitStruct = '5f'  #elev, azim, doppler, range, snr
        pUnitSize = struct.calcsize(pUnitStruct)
        pUnit = struct.unpack(pUnitStruct, data[:pUnitSize])
        data = data[pUnitSize:]
        objStruct = '2bh2H' #2 int8, 1 int16, 2 uint16
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int((tlvLength-pUnitSize)/objSize)
        #print('Parsed Points: ', self.numDetectedObj)
        for i in range(self.numDetectedObj):
            try:
                elev, az, doppler, ran, snr = struct.unpack(objStruct, data[:objSize])
                #print(elev, az, doppler, ran, snr)
                data = data[objSize:]
                #get range, azimuth, doppler, snr
                self.pcPolar[0,i] = ran*pUnit[3]           #range
                if (az >= 128):
                    print ('Az greater than 127')
                    az -= 256
                if (elev >= 128):
                    print ('Elev greater than 127')
                    elev -= 256
                if (doppler >= 32768):
                    print ('Doppler greater than 32768')
                    doppler -= 65536
                self.pcPolar[1,i] = az*pUnit[1]  #azimuth
                self.pcPolar[2,i] = elev*pUnit[0] #elevation
                self.pcPolar[3,i] = doppler*pUnit[2]       #doppler
                self.pcPolar[4,i] = snr*pUnit[4]           #snr
            except:
                self.numDetectedObj = i
                break
        self.polar2Cart3D()

    #decode 2D People Counting Target List TLV
    def parseDetectedTracks(self, data, tlvLength):
        if (self.plotDimension):
            targetStruct = 'I8f9ff'
        else:
            targetStruct = 'I6f9ff'
        targetSize = struct.calcsize(targetStruct)
        self.numDetectedTarget = int(tlvLength/targetSize)
        targets = np.empty((13,self.numDetectedTarget))
        for i in range(self.numDetectedTarget):
            targetData = struct.unpack(targetStruct,data[:targetSize])
            targets[0,i]=int(targetData[0])
            targets[1:3,i]=targetData[1:3]
            targets[3,i]=0
            targets[4:6,i]=targetData[3:5]
            targets[6,i]=0
            targets[7:9,i]=targetData[5:7]
            targets[9,i]=0
            if (self.plotDimension):
                targets[10:12,i]=targetData[7:9]
                targets[12,i]=1
            else:
                targets[10:12,i]=[0.75,0.75]
                targets[12,i]=1
            data = data[targetSize:]    
        self.targetBufPing = targets           

    #decode 3D People Counting Target List TLV
    def parseDetectedTracks3D(self, data, tlvLength):
        targetStruct = 'I9f'
        targetSize = struct.calcsize(targetStruct)
        self.numDetectedTarget = int(tlvLength/targetSize)
        targets = np.empty((13,self.numDetectedTarget))
        for i in range(self.numDetectedTarget):
            targetData = struct.unpack(targetStruct,data[:targetSize])
            targets[0:7,i]=targetData[0:7]
            targets[7:10,i]=[0,0,0]
            targets[10:13,i] = targetData[7:10]
            data = data[targetSize:]
        self.targetBufPing = targets

    #decode Target Index TLV
    def parseTargetAssociations(self, data):
        targetStruct = 'B'
        targetSize = struct.calcsize(targetStruct)
        numIndexes = int(len(data)/targetSize)
        self.indexes = []
        self.unique = []
        for i in range(numIndexes):
            ind = struct.unpack(targetStruct, data[:targetSize])
            self.indexes.append(ind[0])
            data = data[targetSize:]
        if (self.getUnique):
            uTemp = self.indexes[math.ceil(numIndexes/2):]
            self.indexes = self.indexes[:math.ceil(numIndexes/2)]
            for i in range(math.ceil(numIndexes/8)):
                for j in range(8):
                    self.unique.append(getBit(uTemp[i], j))

    #decode Classifier output
    def parseClassifierOutput(self, data):
        classifierDataStruct = 'Ii'
        clOutSize = struct.calcsize(classifierDataStruct)
        self.classifierOutput = np.zeros((2,self.numDetectedTarget))
        for i in range(self.numDetectedTarget):
            self.classifierOutput[0,i], self.classifierOutput[1,i] = struct.unpack(classifierDataStruct, data[:clOutSize])
            data = data[clOutSize:]

#below is for labs that are compliant with SDK 3.x  This code can parse the point cloud TLV and point cloud side info TLV from the OOB demo.  
#It can parse the SDK3.x Compliant People Counting demo "tracker_dpc"
    #get SDK3.x Cartesian Point Cloud
    def parseSDK3xPoints(self, dataIn, numObj):
        pointStruct = '4f'
        pointLength = struct.calcsize(pointStruct)
        try:
            for i in range(numObj):
                self.pcBufPing[0,i], self.pcBufPing[1,i], self.pcBufPing[2,i], self.pcBufPing[3,i] = struct.unpack(pointStruct, dataIn[:pointLength])
                dataIn = dataIn[pointLength:]
            self.pcBufPing = self.pcBufPing[:,:numObj]
        except:
            self.fail = 1

    #get Side Info SDK 3.x
    def parseSDK3xSideInfo(self, dataIn, numObj):
        sideInfoStruct = '2h'
        sideInfoLength = struct.calcsize(sideInfoStruct)
        try:
            for i in range(numObj):
                self.pcBufPing[4,i], unused = struct.unpack(sideInfoStruct, dataIn[:sideInfoLength])
                dataIn = dataIn[sideInfoLength:]
        except:
            self.fail = 1

    #convert SDK compliant Polar Point Cloud to Cartesian
    def polar2CartSDK3(self):
        self.pcBufPing = np.empty((5,self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[2,n] = self.pcPolar[0,n]*math.sin(self.pcPolar[2,n]) #z
            self.pcBufPing[0,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.sin(self.pcPolar[1,n]) #x
            self.pcBufPing[1,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.cos(self.pcPolar[1,n]) #y
        self.pcBufPing[3,:] = self.pcPolar[3,0:self.numDetectedObj] #doppler

    #decode SDK3.x Format Point Cloud in Polar Coordinates
    def parseSDK3xPolar(self, dataIn, tlvLength):
        pointStruct = '4f'
        pointLength = struct.calcsize(pointStruct)
        self.numDetectedObj = int(tlvLength/pointLength)
        try:
            for i in range(self.numDetectedObj):
                self.pcPolar[0,i], self.pcPolar[1,i], self.pcPolar[2,i], self.pcPolar[3,i] = struct.unpack(pointStruct, dataIn[:pointLength])
                dataIn = dataIn[pointLength:]
        except:
            self.fail = 1
            return
        self.polar2CartSDK3()

    #decode 3D People Counting Target List TLV
    def parseDetectedTracksSDK3x(self, data, tlvLength):
        targetStruct = 'I12f'
        targetSize = struct.calcsize(targetStruct)
        self.numDetectedTarget = int(tlvLength/targetSize)
        targets = np.empty((13,self.numDetectedTarget))
        for i in range(self.numDetectedTarget):
            targetData = struct.unpack(targetStruct,data[:targetSize])
            targets[0:13,i]=targetData[0:7]
            data = data[targetSize:]
        self.targetBufPing = targets

    #all TLV header decoding functions are below. Each lab with a Unique header or unique TLV set has its own header parsing function
    #decode Header and rest of TLVs for Legacy Labs and Indoor False detection mitigation
    def tlvHeader(self, data):
        #search for magic word
        self.targetBufPing = np.zeros((12,1))
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.indexes = []
        frameNum = -1
        self.numDetectedTarget = 0
        self.numDetectedObj = 0
        #search until we find magic word
        while (1):
            try:
                magic, version, platform, timestamp, packetLength, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum =  struct.unpack('Q10I2H', data[:self.headerLength])
            except:
                #bad data, return
                self.fail = 1
                return data
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                data = data[1:]
            else:
                #we have correct magic word, proceed to parse rest of data
                break
        if (self.frameNum != frameNum):
            self.missedFrames += 1
            self.frameNum = frameNum
        self.frameNum += 1
        if (len(data) < packetLength):
            ndata = self.dataCom.read(packetLength-len(data))
            self.oldData += ndata
            data += ndata
        data = data[self.headerLength:]
        for i in range(numTLVs):
            try:
                tlvType, tlvLength = self.tlvHeaderDecode(data[:8])
            except:
                print('read fail: not enough data')
                self.missedFrames += 1
                self.fail=1
                break
            try:
                data = data[8:]
                if (tlvType == 6):
                    if(self.threeD):
                        self.parseDetectedObjects3D(data[:tlvLength], tlvLength-8)
                    elif(self.ifdm):
                        self.parseDetectedObjectsIFDM(data[:tlvLength], tlvLength-8)
                    else:
                        self.parseDetectedObjects(data[:tlvLength], tlvLength-8)
                elif (tlvType == 7):
                    if(self.threeD):
                        self.parseDetectedTracks3D(data[:tlvLength], tlvLength-8)
                    else:
                        self.parseDetectedTracks(data[:tlvLength], tlvLength-8)
                elif (tlvType == 8):
                    self.parseTargetAssociations(data[:tlvLength-8])
                elif (tlvType == 9):
                    self.parseClassifierOutput(data[:tlvLength-8])
                data = data[tlvLength-8:]
            except:
                print('Not enough data')
                print('Data length: ', len(data))
                print('Reported Packet Length: ', packetLength)
                self.fail=1
                return data
        return data

    #parsing for SDK 3.x Point Cloud
    def sdk3xTLVHeader(self, dataIn):
        #reset point buffers
        self.pcBufPing = np.zeros((5,self.maxPoints))
        headerStruct = 'Q8I'
        headerLength = struct.calcsize(headerStruct)
        tlvHeaderLength = 8
        #search until we find magic word
        while(1):
            try:
                magic, version, totalPacketLen, platform, self.frameNum, timeCPUCycles, self.numDetectedObj, numTLVs, subFrameNum = struct.unpack(headerStruct, dataIn[:headerLength])
            except:
                #bad data, return
                self.fail = 1
                return dataIn
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                dataIn = dataIn[1:]
            else:
                #we have correct magic word, proceed to parse rest of data
                break
        dataIn = dataIn[headerLength:]
        remainingData = totalPacketLen - len(dataIn)
        #check to ensure we have all of the data
        if (remainingData > 0):
            dataIn += self.dataCom.read(remainingData)
        #now check TLVs
        #print ('got tlvs sdk3x')
        for i in range(numTLVs):
            try:
                tlvType, tlvLength = self.tlvHeaderDecode(dataIn[:tlvHeaderLength])
            except Exception as e:
                print(e)
                print ('failed to read OOB SDK3.x TLV')
            dataIn = dataIn[tlvHeaderLength:]
            if (tlvType == 1):
                self.parseSDK3xPoints(dataIn[:tlvLength], self.numDetectedObj)
                dataIn = dataIn[tlvLength:]
            elif (tlvType == 7):
                self.parseSDK3xSideInfo(dataIn[:tlvLength], self.numDetectedObj)
                dataIn = dataIn[tlvLength:]




    #parsing for SDK 3.x DPIF compliant People Counting
    def sdk3xPCHeader(self, dataIn):
        #reset point buffers
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.targetBufPing = np.zeros((13,20))
        self.indexes = []
        tlvHeaderLength = 8
        #search until we find magic word
        while (1):
            try:
                magic, version, platform, timestamp, packetLength, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum =  struct.unpack('Q10I2H', dataIn[:self.headerLength])
            except:
                #bad data, return
                self.fail = 1
                return dataIn
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                dataIn = dataIn[1:]
            else:
                #we have correct magic word, proceed to parse rest of data
                break
        dataIn = dataIn[self.headerLength:]
        remainingData = packetLength - len(dataIn)
        print('pl: ', packetLength)
        print('remainingData ', remainingData)
        #check to ensure we have all of the data
        if (remainingData > 0):
            dataIn += self.dataCom.read(remainingData)
        #now check TLVs
        for i in range(numTLVs):
            try:
                #print("DataIn Type", type(dataIn))
                tlvType, tlvLength = self.tlvHeaderDecode(dataIn[:tlvHeaderLength])
            except Exception as e:
                print(e)
                print ('failed to read OOB SDK3.x TLV')
            dataIn = dataIn[tlvHeaderLength:]
            dataLength = tlvLength
            if (tlvType == 6):
                #DPIF Polar Coordinates
                self.parseSDK3xPolar(dataIn[:dataLength], dataLength)
            elif (tlvType == 7):
                #target 3D
                self.parseDetectedTracks3D(dataIn[:dataLength], dataLength)
            elif (tlvType == 8):
                #target index
                self.parseTargetAssociations(dataIn[:dataLength])
            elif (tlvType == 9):
                #side info
                self.parseSDK3xSideInfo(dataIn[:dataLength], self.numDetectedObj)
            dataIn = dataIn[dataLength:]

    #parsing for 3D People Counting lab
    def Capon3DHeader(self, dataIn):
        #reset point buffers
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.pcPolar = np.zeros((5,self.maxPoints))
        self.targetBufPing = np.zeros((13,20))
        self.numDetectedTarget = 0
        self.numDetectedObj = 0
        self.indexes = []
        tlvHeaderLength = 8
        headerLength = 48
        #stay in this loop until we find the magic word or run out of data to parse
        while (1):
            try:
                magic, version, packetLength, platform, self.frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum =  struct.unpack('Q9I2H', dataIn[:headerLength])
            except:
                #bad data, return
                self.fail = 1
                return dataIn
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                dataIn = dataIn[1:]
            else:
                #got magic word, proceed to parse
                break
        dataIn = dataIn[headerLength:]
        remainingData = packetLength - len(dataIn) - headerLength
        #check to ensure we have all of the data
        if (remainingData > 0):
            newData = self.dataCom.read(remainingData)
            dataIn += newData
            self.oldData += newData
        #now check TLVs
        for i in range(numTLVs):
            try:
                #print("DataIn Type", type(dataIn))
                tlvType, tlvLength = self.tlvHeaderDecode(dataIn[:tlvHeaderLength])
            except Exception as e:
                print(e)
                print ('failed to read OOB SDK3.x TLV')
            dataIn = dataIn[tlvHeaderLength:]
            dataLength = tlvLength-tlvHeaderLength
            if (tlvType == 6):
                #DPIF Polar Coordinates
                self.parseCapon3DPolar(dataIn[:dataLength], dataLength)
            elif (tlvType == 7):
                #target 3D
                self.parseDetectedTracks3D(dataIn[:dataLength], dataLength)
            elif (tlvType == 8):
                #target index
                self.parseTargetAssociations(dataIn[:dataLength])
            elif (tlvType == 9):
                print('type9')
                #side info
                #self.parseSDK3xSideInfo(dataIn[:dataLength], self.numDetectedObj)
            dataIn = dataIn[dataLength:]
        return dataIn


    # This function is always called - first read the UART, then call a function to parse the specific demo output
    # This will return 1 frame of data. This must be called for each frame of data that is expected. It will return a dict containing:
    #   1. Point Cloud
    #   2. Target List
    #   3. Target Indexes
    #   4. number of detected points in point cloud
    #   5. number of detected targets
    #   6. frame number
    #   7. Fail - if one, data is bad
    #   8. classifier output
    # Point Cloud and Target structure are liable to change based on the lab. Output is always cartesian.
    def readAndParseUart(self):
        self.fail = 0
        if (self.replay):
            return self.replayHist()
        numBytes = 4666
        data = self.dataCom.read(numBytes)
        if (self.byteData is None):
            self.byteData = data
        else:
            self.byteData += data
        self.oldData += data
        try:
            if (self.SDK3xPointCloud == 1):
                self.byteData = self.sdk3xTLVHeader(self.byteData)
            elif (self.SDK3xPC == 1):
                self.byteData = self.sdk3xPCHeader(self.byteData)
            elif (self.capon3D == 1):
                self.byteData = self.Capon3DHeader(self.byteData)
            else:
                self.byteData = self.tlvHeader(self.byteData)
        except:
            self.fail = 1
        #return data after parsing and save to replay file
        if (self.fail):
            return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, self.numDetectedTarget, self.frameNum, self.fail, self.classifierOutput
        if (self.saveBinary):
            if (self.frameNum%1000 == 0):
                toSave = bytes(self.oldData)
                fileName = 'binData/pHistBytes_'+str(self.saveNum)+'.bin'
                self.saveNum += 1
                bfile = open(fileName, 'wb')
                bfile.write(toSave)
                self.oldData = []
                print ('Missed Frames ' + str(self.missedFrames/1000))
                self.missedFrames = 0
        parseEnd = int(round(time.time()*1000))
        return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, self.numDetectedTarget, self.frameNum, self.fail, self.classifierOutput

    #find various utility functions here for connecting to COM Ports, send data, etc...
    #connect to com ports
    # Call this function to connect to the comport. This takes arguments self (intrinsic), uartCom, and dataCom. No return, but sets internal variables in the parser object.
    def connectComPorts(self, uartCom, dataCom):
        self.uartCom = serial.Serial(uartCom, 115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.3)
        if (self.capon3D == 1 and self.aop == 0):
            self.dataCom = serial.Serial(dataCom, 921600*2,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.025)
        else:
            self.dataCom = serial.Serial(dataCom, 921600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.025)
        self.dataCom.reset_output_buffer()
        print('Connected')

    #send cfg over uart
    def sendCfg(self, cfg):
        for line in cfg:
            time.sleep(.03)
            self.uartCom.write(line.encode())
            ack = self.uartCom.readline()
            print(ack)
            ack = self.uartCom.readline()
            print(ack)
        time.sleep(3)
        self.uartCom.reset_input_buffer()

    #send single command to device over UART Com.
    def sendLine(self, line):
        self.uartCom.write(line.encode())
        ack = self.uartCom.readline()
        print(ack)
        ack = self.uartCom.readline()
        print(ack)

    def replayHist(self):
        if (self.replayData):
            frameData = self.replayData[0]
            self.replayData = self.replayData[1:]
            return frameData['PointCloud'], frameData['Targets'], frameData['Indexes'], frameData['Number Points'], frameData['NumberTracks'],frameData['frame'],0, frameData['ClassifierOutput'], frameData['Uniqueness']
        else:
            filename = '1Person1FanClass3NoGhostsTilEnd_AkashEntersScene/pHistRT'+str(self.saveNum)+'.pkl'
            #filename = 'Replay1Person10mShort/pHistRT'+str(self.saveNum)+'.pkl'
            self.saveNum+=1
            try:
                dfile = open(filename, 'rb')
            except:
                return -1
            self.replayData = pickle.load(dfile)
            if (self.replayData):
                print('entering replay')
                return self.replayHist()
            else:
                return -1
        
def getBit(byte, bitNum):
    mask = 1 << bitNum
    if (byte&mask):
        return 1
    else:
        return 0