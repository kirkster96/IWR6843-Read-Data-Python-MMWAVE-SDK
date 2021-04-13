# ****************************************************************************
# * (C) Copyright 2020, Texas Instruments Incorporated. - www.ti.com
# ****************************************************************************
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions are
# *  met:
# *
# *    Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# *    Redistributions in binary form must reproduce the above copyright
# *    notice, this list of conditions and the following disclaimer in the
# *     documentation and/or other materials provided with the distribution.
# *
# *    Neither the name of Texas Instruments Incorporated nor the names of its
# *    contributors may be used to endorse or promote products derived from
# *    this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# *  PARTICULAR TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# *  A PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR
# *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# *  EXEMPLARY, ORCONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# *  LIABILITY, WHETHER IN CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING
# *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *

# import the required Python packages
import os
import sys
import struct
import math
import binascii
import codecs

# Global variables hold test configurations
# Set one time per test by read_config() function when read configuration parameters from test profile file
# Used by parser processing 
CFG_PARAMS = {}
 
numOfChirps_buf = []
numLoops_buf = []
numAdcSamples_buf = []
profileIdx_buf = []
SigImgNumSlices_buf = []
RxSatNumSlices_buf = []
chanIdx_buf = []

lvdsCfg_headerEn_buf = []
lvdsCfg_dataFmt_buf = []
lvdsCfg_userBufEn_buf = []

Raw_file_numSubframes = 0
Raw_file_subframeIdx_buf = []
Raw_file_sessionFlag = ""

ADC_file_numSubframes = 0
ADC_file_subframeIdx_buf = []
ADC_file_sessionFlag = ""

CC9_file_numSubframes = 0
CC9_file_subframeIdx_buf = []
CC9_file_sessionFlag = "" 

# Global variables hold lvds hsi header parameters
# Write by get_hsi_header() function
# Read and verify by verify_hsi_header_hw() and verify_hsi_header_sw() functions      
HSI_HEADER = {}
HSI_HEADER_backup = {}

# Definations for test pass/fail or not_apply/don't care
NOT_APPLY = -1
TC_PASS   =  0
TC_FAIL   =  1

def getUint32(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer.

        @param data : 1-demension byte array  
        @return     : 32-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256 +
            data[2]*65536 +
            data[3]*16777216)

def getUint16(data):
    """!
       This function coverts 2 bytes to a 16-bit unsigned integer.

        @param data : 1-demension byte array
        @return     : 16-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256)

def get_hsi_header(fp):
    """!
       This function get HSI header in the captured file
 
        @param fp : captured file handler
        @return None
    """
    global HSI_HEADER
    global HSI_HEADER_backup
    global zero_fill_flag_packet # 0 is non-zero-filled packet. 1 is zero-filled packet

    #print("get_hsi_header()") 
    
    zero_fill_flag_packet = 0 # init to non-zero-filled packet  

    realHeaderSize_Byte = 44 # fixed value based on data structure defined 

    # Read Header
    HSI_HEADER['id']              = struct.unpack('<Q',fp.read(8))
    HSI_HEADER['totalLengthLSW']  = struct.unpack('<H',fp.read(2))
    HSI_HEADER['totalLengthMSW']  = struct.unpack('<H',fp.read(2)) 
    HSI_HEADER['reserved']        = struct.unpack('<I',fp.read(4))
    HSI_HEADER['version']         = struct.unpack('<H',fp.read(2))
    HSI_HEADER['headerSize']      = struct.unpack('<H',fp.read(2)) 
    HSI_HEADER['platform']        = struct.unpack('<B',fp.read(1)) 
    HSI_HEADER['interleavedMode'] = struct.unpack('<B',fp.read(1)) 
    HSI_HEADER['dataSize']        = struct.unpack('<B',fp.read(1)) 
    HSI_HEADER['dataType']        = struct.unpack('<B',fp.read(1)) 
    HSI_HEADER['rxChannelStatus'] = struct.unpack('<B',fp.read(1)) 
    HSI_HEADER['dataFmt']         = struct.unpack('<B',fp.read(1))     
    HSI_HEADER['chirpMode']       = struct.unpack('<H',fp.read(2)) 
    HSI_HEADER['adcDataSize']     = struct.unpack('<H',fp.read(2)) 
    HSI_HEADER['cpDataSize']      = struct.unpack('<H',fp.read(2)) 
    HSI_HEADER['cqDataSize']      = struct.unpack('<3H',fp.read(2*3)) 
    HSI_HEADER['userBufSize']     = struct.unpack('<3H',fp.read(2*3)) 

    #print(HSI_HEADER)

    # Check if the packet zero filled
    if HSI_HEADER['headerSize'][0] <= 0: 
        # zero-filled packet
        print("\n*** This is a zero-filled packet *** headerSize = %d\n" % (HSI_HEADER['headerSize'][0]))
        zero_fill_flag_packet = 1

        # Copy HSI_HEADER_backup to HSI_HEADER
        HSI_HEADER['id']              = HSI_HEADER_backup['id']
        HSI_HEADER['totalLengthLSW']  = HSI_HEADER_backup['totalLengthLSW']
        HSI_HEADER['totalLengthMSW']  = HSI_HEADER_backup['totalLengthMSW']
        HSI_HEADER['reserved']        = HSI_HEADER_backup['reserved']
        HSI_HEADER['version']         = HSI_HEADER_backup['version'] 
        HSI_HEADER['headerSize']      = HSI_HEADER_backup['headerSize'] 
        HSI_HEADER['platform']        = HSI_HEADER_backup['platform'] 
        HSI_HEADER['interleavedMode'] = HSI_HEADER_backup['interleavedMode'] 
        HSI_HEADER['dataSize']        = HSI_HEADER_backup['dataSize'] 
        HSI_HEADER['dataType']        = HSI_HEADER_backup['dataType']
        HSI_HEADER['rxChannelStatus'] = HSI_HEADER_backup['rxChannelStatus']
        HSI_HEADER['dataFmt']         = HSI_HEADER_backup['dataFmt']     
        HSI_HEADER['chirpMode']       = HSI_HEADER_backup['chirpMode'] 
        HSI_HEADER['adcDataSize']     = HSI_HEADER_backup['adcDataSize'] 
        HSI_HEADER['cpDataSize']      = HSI_HEADER_backup['cpDataSize']
        HSI_HEADER['cqDataSize']      = HSI_HEADER_backup['cqDataSize'] 
        HSI_HEADER['userBufSize']     = HSI_HEADER_backup['userBufSize']

        #print(HSI_HEADER)
    else:
        # non-zero-filled packet
        # Save HSI_HEADER to HSI_HEADER_backup which will be used if the next packet is a zero-filled packet
        HSI_HEADER_backup['id']              = HSI_HEADER['id']
        HSI_HEADER_backup['totalLengthLSW']  = HSI_HEADER['totalLengthLSW']
        HSI_HEADER_backup['totalLengthMSW']  = HSI_HEADER['totalLengthMSW']
        HSI_HEADER_backup['reserved']        = HSI_HEADER['reserved']
        HSI_HEADER_backup['version']         = HSI_HEADER['version'] 
        HSI_HEADER_backup['headerSize']      = HSI_HEADER['headerSize'] 
        HSI_HEADER_backup['platform']        = HSI_HEADER['platform'] 
        HSI_HEADER_backup['interleavedMode'] = HSI_HEADER['interleavedMode'] 
        HSI_HEADER_backup['dataSize']        = HSI_HEADER['dataSize'] 
        HSI_HEADER_backup['dataType']        = HSI_HEADER['dataType']
        HSI_HEADER_backup['rxChannelStatus'] = HSI_HEADER['rxChannelStatus']
        HSI_HEADER_backup['dataFmt']         = HSI_HEADER['dataFmt']     
        HSI_HEADER_backup['chirpMode']       = HSI_HEADER['chirpMode'] 
        HSI_HEADER_backup['adcDataSize']     = HSI_HEADER['adcDataSize'] 
        HSI_HEADER_backup['cpDataSize']      = HSI_HEADER['cpDataSize']
        HSI_HEADER_backup['cqDataSize']      = HSI_HEADER['cqDataSize'] 
        HSI_HEADER_backup['userBufSize']     = HSI_HEADER['userBufSize']

    # Read padding
    headerSize_Byte = HSI_HEADER['headerSize'][0] * 2 # 1 CBuff unit = 2 bytes
    paddingSize_Byte = headerSize_Byte - realHeaderSize_Byte
    fmt = '<%dB' % (paddingSize_Byte)
    paddingBuffer   = struct.unpack(fmt,fp.read(paddingSize_Byte))


def verify_hsi_header_hw(numAdcSamples, ID):
    """!
       This function verify HW session's HSI header in the captured file

        @param numAdcSamples : number of ADC samples
        @param ID : session ID could be 926064613602757340(0xCDA 0ADC 0CDA 0ADC) or 705953299182652617(0x9CC 0CC9 09CC 0CC9) 
        @return TC_PASS : correct
        @return TC_FAIL : wrong 
    """
    global CFG_PARAMS    
    global HSI_HEADER

    #print("verify_hsi_header_hw()")   
       
    #print(HSI_HEADER)
 
    # Verify Header 
    if (HSI_HEADER['id'][0] != ID):
        print("\n*** Error: Wrong hw session ID ***\n")
        return TC_FAIL

    if HSI_HEADER['platform'][0] == 1:#XWR14XX 
        if '14' not in CFG_PARAMS['platfrom']:
            print("\n*** Error: Wrong platform ***\n") 
            return TC_FAIL      
    elif HSI_HEADER['platform'][0] == 2:#XWR16XX
        if '16' not in CFG_PARAMS['platfrom']:
            print("\n*** Error: Wrong platform ***\n")
            return TC_FAIL 
    elif HSI_HEADER['platform'][0] == 3:#XWR18XX
        if '18' not in CFG_PARAMS['platfrom']:
            print("\n*** Error: Wrong platform ***\n")
            return TC_FAIL
    elif HSI_HEADER['platform'][0] == 4:#XWR68XX
        if '68' not in CFG_PARAMS['platfrom'] and '64' not in CFG_PARAMS['platfrom']:
            print("\n*** Error: Wrong platform ***\n")
            return TC_FAIL
    else:
        print("\n*** Error: Undefined platform ***\n")
        return TC_FAIL

    if HSI_HEADER['platform'][0] == 1:#XWR14XX
        if HSI_HEADER['interleavedMode'][0] != 1:
            print("\n*** Error: Wrong interleavedMode ***\n")
            return TC_FAIL
    else: 
        if HSI_HEADER['interleavedMode'][0] != 2:
            print("\n*** Error: Wrong interleavedMode ***\n")
            return TC_FAIL
    
    if HSI_HEADER['dataSize'][0] == 0:# 16 bit
        if CFG_PARAMS['dataSize'] != 2:
            print("\n*** Error: Wrong dataSize ***\n")
            return TC_FAIL
    elif HSI_HEADER['dataSize'][0] == 1:# 14 bit
        if CFG_PARAMS['dataSize'] != 1:
            print("\n*** Error: Wrong dataSize ***\n")
            return TC_FAIL
    elif HSI_HEADER['dataSize'][0] == 2:# 12 bit
        if CFG_PARAMS['dataSize'] != 0:
            print("\n*** Error: Wrong dataSize ***\n")
            return TC_FAIL
    else:
        print("\n*** Error: Undefined dataSize ***\n")
        return TC_FAIL

    if (HSI_HEADER['dataType'][0] != CFG_PARAMS['dataType'] + 1):
        print("\n*** Error: Wrong dataType ***\n") 
        return TC_FAIL

    if (HSI_HEADER['rxChannelStatus'][0] != CFG_PARAMS['rxAntMask']):
        print("\n*** Error: Wrong rxChannelStatus ***\n") 
        return TC_FAIL

    # must match with defination in mmwave_sdk/packages/ti/utils/hsiheader/hsiprotocol.h
    if (HSI_HEADER['dataFmt'][0] != CFG_PARAMS['dataFmt']):
        print("\n*** Error: Wrong dataFmt ***\n")
        return TC_FAIL
        
    if CFG_PARAMS['chirpMode'] == 0: #multi-chirp
        if (HSI_HEADER['chirpMode'][0] < 2):
            print("\n*** Error: Wrong chirpMode ***\n")
            return TC_FAIL
    if CFG_PARAMS['chirpMode'] == 1: #single-chirp
        if (HSI_HEADER['chirpMode'][0] != 1):
            print("\n*** Error: Wrong chirpMode ***\n")
            return TC_FAIL 

    if (HSI_HEADER['adcDataSize'][0] != numAdcSamples*(CFG_PARAMS['dataType'] + 1)):
        print("\n*** Error: Wrong adcDataSize ***\n")
        return TC_FAIL

    return TC_PASS


def verify_hsi_header_sw(fp, ID, frameIdx, subframeIdx):
    """!
       This function verify SW session's HSI header in the captured file

        @param fp : captured file handler 
        @param ID : session ID could be 926064613602757340(0xCDA 0ADC 0CDA 0ADC) or 705953299182652617(0x9CC 0CC9 09CC 0CC9) 
        @return TC_PASS : correct
        @return TC_FAIL : wrong 
    """
    global HSI_HEADER
    global userBuf0
    global userBuf1
    global userBuf2

    #print("verify_hsi_header_sw()") 
    
    #print(HSI_HEADER)
 
    # Verify Header 
    if (HSI_HEADER['id'][0] != ID):
        print("\n*** Error: Wrong sw session ID ***\n")
        return TC_FAIL

    if HSI_HEADER['platform'][0] == 1:#XWR14XX 
        if '14' not in CFG_PARAMS['platfrom']:
            print("\n*** Error: Wrong platform ***\n") 
            return TC_FAIL      
    elif HSI_HEADER['platform'][0] == 2:#XWR16XX
        if '16' not in CFG_PARAMS['platfrom']:
            print("\n*** Error: Wrong platform ***\n")
            return TC_FAIL 
    elif HSI_HEADER['platform'][0] == 3:#XWR18XX
        if '18' not in CFG_PARAMS['platfrom']:
            print("\n*** Error: Wrong platform ***\n")
            return TC_FAIL
    elif HSI_HEADER['platform'][0] == 4:#XWR68XX
        if '68' not in CFG_PARAMS['platfrom'] and '64' not in CFG_PARAMS['platfrom']:
            print("\n*** Error: Wrong platform ***\n")
            return TC_FAIL
    else:
        print("\n*** Error: Undefined platform ***\n")
        return TC_FAIL

    # Verify header
    if (HSI_HEADER['userBufSize'][0] != 4):
        print("\n*** Error: Wrong userBufSize0 ***\n")
        return TC_FAIL
    else:
        # Read User Buffer 0
        fmt = '<%dB' % (HSI_HEADER['userBufSize'][0]*2)
        userBuf0 = struct.unpack(fmt,fp.read(HSI_HEADER['userBufSize'][0]*2))
        #print("\nmmw demo userBuf0")
        #print(userBuf0)

        # User Buffer 0 is a byte array 
        # byte0 to byte3 represent a 32-bit integer. Convert byte0 to byte3 to 32-bit integer        
        numFrames = getUint32(userBuf0[0:4:1])
        # byte4 to byte5 represent a 16-bit integer. Convert byte4 to byte5 to 16-bit integer
        subFrameNum = getUint16(userBuf0[4:6:1])
        # byte6 to byte7 represent a 16-bit integer. Convert byte4 to byte5 to 16-bit integer
        numObj = getUint16(userBuf0[6:8:1])
        print("\nmmw Demo userBuf0: numFrames = %d, subFrameNum =  %d, numObj = %d\n" %(numFrames, subFrameNum, numObj))

        # Verify userBuffer0 
        if numFrames != frameIdx + 1:
            print("\n*** Error: Wrong userBuf0.numFrames ***\n")
            return TC_FAIL 
        if subFrameNum != subframeIdx:
            print("\n*** Error: Wrong userBuf0.subFrameNum ***\n")
            return TC_FAIL
 
        # Verify userBuffer1 and userBuffer2 size based on numObj in userBuffer0 
        if HSI_HEADER['userBufSize'][1] != numObj * 8:#user buffer 1 contains cloud point per detected obj
            print("\n*** Error: Wrong userBufSize1 ***\n")
            return TC_FAIL
        if HSI_HEADER['userBufSize'][2] != numObj * 2:#user buffer 2 contains snr and noise per detected obj
            print("\n*** Error: Wrong userBufSize2 ***\n")
            return TC_FAIL
            
        # Read User Buffer 1
        fmt = '<%dB' % (HSI_HEADER['userBufSize'][1]*2)
        userBuf1 = struct.unpack(fmt,fp.read(HSI_HEADER['userBufSize'][1]*2))
        #print("\nmmw demo userBuf1")
        #print(userBuf1)

        # Read User Buffer 2
        fmt = '<%dB' % (HSI_HEADER['userBufSize'][2]*2)
        userBuf2 = struct.unpack(fmt,fp.read(HSI_HEADER['userBufSize'][2]*2))
        #print("\nmmw demo userBuf2")
        #print(userBuf2)
                                                  
    return TC_PASS


def get_ADC(fp, numAdcSamples, dataSize):
    """!
       This function get raw ADC samples from the captured file

        @param fp : captured file handler
        @param numAdcSamples : number of ADC samples
        @param dataSize : ADC sample size 16bit, 14bit or 12bit
    """
    global ADC_buffer
    
    #print("get_ADC()")              
    # Read ADC samples in 16-bit signed int
    adcDataSize = numAdcSamples*(CFG_PARAMS['dataType'] + 1)
    fmt = '<%dh' % (adcDataSize) # 16-bit signed int = 2 bytes
    ADC_buffer = struct.unpack(fmt,fp.read(adcDataSize*2))
    
    #print("ADC_buffer")
    #print(ADC_buffer)

    # Convert to signed values if 12bit or 14bit dataSize
    if dataSize == 0 or dataSize == 1: # 12bit or 14bit
        # 2's complement convert for negtive value
        if dataSize == 1:#14bit
            max_positive_value = 8191
            mask = 0x1fff
        if dataSize == 0:#12bit
            max_positive_value = 2047
            mask = 0x7ff 

        for i in range(numAdcSamples):
            for ii in range(2):
                if ADC_buffer[i*2+ii] > max_positive_value: #negtive value 
                    ADC_buffer[i*2+ii] = (ADC_buffer[i*2+ii] & mask) - (max_positive_value + 1) #2's complement
    #end if 12bit or 14bit

def get_verify_CP(fp, cp0, cp1):  
    """!
       This function get and verify CP contained in the captured file

        @param fp : captured file handler
        @param cp0 : golden cp0 value
        @param cp1 : golden cp1 value
        @return verify_result : pass or fail
        @return CP_buffer: CP buffer
    """
    global CFG_PARAMS    
    global HSI_HEADER
    global zero_fill_flag_packet

    #print("get_verify_CP()")

    # hardcode in no header case
    cpDataSize = 2
    if CFG_PARAMS['headerEn'] == 1:
        cpDataSize = HSI_HEADER['cpDataSize'][0] # CBuff unit = 2 bytes
    
    if cpDataSize > 0:
        fmt = '<%dh' % (cpDataSize) # 16-bit signed int = 2 bytes
        CP_buffer = struct.unpack(fmt,fp.read(cpDataSize*2))
        #print(CP_buffer)
    
    verify_result = TC_PASS

    # Verify CP only for non-zero-filled packet
    # Notes: a non-zero-filled packet which is right before a zero-filled packet may have CP = 0.
    if zero_fill_flag_packet == 0:
        if CP_buffer[0] != cp0:
            if CP_buffer[0] == 0:
                print("\n*** CP[0] = 0 ***\n") 
            else:
                print("\n*** Error: Wrong CP[0] ***\n")
                verify_result = TC_FAIL
        if CP_buffer[1] != cp1:
            if CP_buffer[1] == 0:
                print("\n*** CP[1] = 0 ***\n")
            else:
                print("\n*** Error: Wrong CP[1] ***\n")
                verify_result = TC_FAIL

    return (verify_result, CP_buffer)

def get_verify_CQ(fp, CQ1N, CQ2N):
    """!
       This function get and verify CQ contained in the captured file

        @param fp : captured file handler
        @param CQ1N : golden CQ1 N value
        @param CQ2N : golden CQ2 N value
        @return verify_result : pass or fail
        @return CQ1_buffer: CQ1 buffer
        @return CQ2_buffer: CQ2 buffer
    """
    global CFG_PARAMS 
    global HSI_HEADER
    global zero_fill_flag_packet
    
    #print("get_verify_CQ()")
    #print(CQ1N)
    #print(CQ2N)

    cqDataSize = HSI_HEADER['cqDataSize'][0] # CBuff unit = 2 bytes
    if cqDataSize > 0: 
        fmt = '<%dh' % (cqDataSize) # 16-bit signed int = 2 bytes
        CQ0_buffer = struct.unpack(fmt,fp.read(cqDataSize*2))
        #print("CQ0_buffer")
        #print(CQ0_buffer)

    cqDataSize = HSI_HEADER['cqDataSize'][1] 
    if cqDataSize > 0:
        fmt = '<%dh' % (cqDataSize)
        CQ1_buffer = struct.unpack(fmt,fp.read(cqDataSize*2))
        #print("CQ1_buffer")
        #print(CQ1_buffer)

    cqDataSize = HSI_HEADER['cqDataSize'][2]
    if cqDataSize > 0: 
        fmt = '<%dh' % (cqDataSize) 
        CQ2_buffer = struct.unpack(fmt,fp.read(cqDataSize*2))
        #print("CQ2_buffer")
        #print(CQ2_buffer)

    verify_result = TC_PASS

    # Verify CQ only for non-zero-filled packet
    # Notes: a non-zero-filled packet which is right before a zero-filled packet may have CQ = 0.
    if zero_fill_flag_packet == 0:
        if CQ1_buffer[0] != CQ1N:
            if CQ1_buffer[0] == 0:
                print("\n*** CQ1[0] = 0 ***\n")
            else:
                print("\n*** Error: Wrong CQ1 N value ***\n")
                verify_result = TC_FAIL
        if CQ2_buffer[0] != CQ2N:
            if CQ2_buffer[0] == 0:
                print("\n*** CQ2[0] = 0 ***\n")
            else: 
                print("CQ2[0] = %d  CQ2N = %d\n" % (CQ2_buffer[0], CQ2N))
                print("\n*** Error: Wrong CQ2 N value ***\n")
                verify_result = TC_FAIL
    
    return (verify_result, CQ1_buffer, CQ2_buffer)


def read_config (config_file_name):
    """!
       This function read config from test profile file and fills up global variables to contain the configuration

        @param config_file_name : test config profile file name
        @return None
    """
    global CFG_PARAMS
 
    global numOfChirps_buf
    global numLoops_buf
    global numAdcSamples_buf
    global profileIdx_buf
    global SigImgNumSlices_buf
    global RxSatNumSlices_buf
    global chanIdx_buf

    global lvdsCfg_headerEn_buf
    global lvdsCfg_dataFmt_buf
    global lvdsCfg_userBufEn_buf

    global Raw_file_numSubframes
    global Raw_file_subframeIdx_buf
    global Raw_file_sessionFlag

    global ADC_file_numSubframes
    global ADC_file_subframeIdx_buf
    global ADC_file_sessionFlag

    global CC9_file_numSubframes
    global CC9_file_subframeIdx_buf
    global CC9_file_sessionFlag

    config = open(config_file_name,'r')

    for line in config:
        print("**** line from config file: \n" + line)
        List = line.split()
                        
        if 'channelCfg' in line:
            CFG_PARAMS['rxAntMask'] = int(List[1])  
        if 'adcCfg' in line:
            CFG_PARAMS['dataSize'] = int(List[1])  
            CFG_PARAMS['dataType'] = int(List[2])  
        if 'adcbufCfg' in line:
            CFG_PARAMS['chirpMode'] = int(List[5])
        if 'Platform' in line:
            if '14' in line:
                CFG_PARAMS['platfrom'] = '14'
            if '16' in line:
                CFG_PARAMS['platfrom'] = '16'
            if '18' in line:
                CFG_PARAMS['platfrom'] = '18'
            if '64' in line:
                CFG_PARAMS['platfrom'] = '64'
            if '68' in line:
                CFG_PARAMS['platfrom'] = '68'

        if 'profileCfg' in line:
            profileIdx_buf.append(int(List[1]))
            numAdcSamples_buf.append(int(List[10])) 
                          
        if 'frameCfg' in line:
            CFG_PARAMS['chirpStartIdx'] = int(List[1]) 
            CFG_PARAMS['chirpEndIdx'] = int(List[2])  
            numOfChirps_buf.append(CFG_PARAMS['chirpEndIdx'] - CFG_PARAMS['chirpStartIdx'] + 1)
            numLoops_buf.append(int(List[3]))
            CFG_PARAMS['numSubframes'] = 1 
                    
        if 'advFrameCfg' in line:
            CFG_PARAMS['numSubframes'] = int(List[1])  
        if 'subFrameCfg' in line:
            numOfChirps_buf.append(int(List[4]))
            numLoops_buf.append(int(List[5]))
                        
        if 'lvdsStreamCfg' in line: 
            lvdsCfg_headerEn_buf.append(int(List[2]))
            lvdsCfg_dataFmt_buf.append(int(List[3]))
            lvdsCfg_userBufEn_buf.append(int(List[4]))

        if 'CQSigImgMonitor' in line: 
            SigImgNumSlices_buf.append(int(List[2]))

        if 'CQRxSatMonitor' in line: 
            RxSatNumSlices_buf.append(int(List[4]))
                          
    config.close()

    ####################################
    ######## Parser rxAnt config #######
    ####################################
    rxAntMask = CFG_PARAMS['rxAntMask']
                
    rxChanEn = []
    rxChanEn.append(rxAntMask & 1)
    rxChanEn.append((rxAntMask >> 1) & 1)
    rxChanEn.append((rxAntMask >> 2) & 1)
    rxChanEn.append((rxAntMask >> 3) & 1)
    #print(rxChanEn) 

    numRxChan = 0
    chanIdx_buf = []
    for chanIdx in range (4):
        if rxChanEn[chanIdx] == 1:
            chanIdx_buf.append(chanIdx)
            numRxChan = numRxChan + 1
    CFG_PARAMS['numRxChan'] = numRxChan
 
    ####################################
    ######## Parser lvds config ########
    ####################################
    Raw_file_numSubframes = 0
    Raw_file_subframeIdx_buf = []
    Raw_file_sessionFlag = ""

    ADC_file_numSubframes = 0
    ADC_file_subframeIdx_buf = []
    ADC_file_sessionFlag = ""

    CC9_file_numSubframes = 0
    CC9_file_subframeIdx_buf = []
    CC9_file_sessionFlag = ""

    # Based on the 1st subframe's lvdsStreamCfg CLI (headerEn, dataFmt and userBufEn)
 
    # if the 1st subframe has no header (headerEn = 0): 
    # > in this case, HW session ADC only (dataFmt = 1) and no SW session (userBufEn = 0) is the only valid configuration combination. 
    # > <prefix>_Raw_<x>.bin is generated to record HW session of the 1st subframe.
    # > in advanced subframe case, rest 3 subframes must have same lvdsStreamCfg as the 1st subframe, and record to <prefix>_Raw_<x>.bin as well. 

    # if the 1st subframe has header (headerEn = 1) and HW session is ADC only (dataFmt = 1) or CP+ADC+CQ (dataFmt = 4): 
    # > <prefix>_hdr_0ADC_<x>.bin is generated to record HW session of the 1st subframe. in advanced subframe case if any of rest 3 subfrmes has HW session, will be recorded to <prefix>_hdr_0ADC_<x>.bin as well. 
    # > <prefix>_hdr_0CC9_<x>.bin will be generated to record SW session if any subframes has SW session (userBufEn = 1). 
    
    # if the 1st subframe has header (headerEn = 1) and no HW session (dataFmt = 0): 
    # > in this case, the 1st subframe must have SW session (userBufEn = 1)
    # > <prefix>_hdr_0ADC_<x>.bin is generated to record SW session of the 1st subframe. In advanced subframe case if any of rest 3 subframes has SW session, will be recorded to <prefix>_hdr_0ADC_<x>.bin as well.  
    # > in advanced subframe case <prefix>_hdr_0CC9_<x>.bin will be generated to record HW session if any of rest 3 subframes has HW session (dataFmt = 1 or dataFmt = 4). 
    
    CFG_PARAMS['datacard_dataLoggingMode'] = "multi"
    if lvdsCfg_headerEn_buf[0] == 0:
        CFG_PARAMS['datacard_dataLoggingMode'] = "raw"

    if lvdsCfg_headerEn_buf[0] == 0:
        if lvdsCfg_dataFmt_buf[0] == 1 and lvdsCfg_userBufEn_buf[0] == 0:
            if CFG_PARAMS['datacard_dataLoggingMode'] == "raw":
                # Raw file
                Raw_file_numSubframes = Raw_file_numSubframes + 1
                Raw_file_subframeIdx_buf.append(0)
                Raw_file_sessionFlag = "HW"
            elif CFG_PARAMS['datacard_dataLoggingMode'] == "multi":
                returen_value = TC_FAIL
                print ("Error: no header can not be in multi mode!")
            else:
                returen_value = TC_FAIL
                print ("Error: Undefined CFG_PARAMS['datacard_dataLoggingMode']!") 
        else:
            returen_value = TC_FAIL
            print ("Error: Subframe %d has a invalid lvdsStreamCfg" % subframeIdx)
    elif lvdsCfg_headerEn_buf[0] == 1:
        if lvdsCfg_dataFmt_buf[0] == 1 or lvdsCfg_dataFmt_buf[0] == 4: # 1:ADC 4:CP+ADC+CQ
            ADC_file_sessionFlag = "HW"
            CC9_file_sessionFlag = "SW"
            ADC_file_numSubframes = ADC_file_numSubframes + 1
            ADC_file_subframeIdx_buf.append(0) 
            if lvdsCfg_userBufEn_buf[0] == 1:
                CC9_file_numSubframes = CC9_file_numSubframes + 1
                CC9_file_subframeIdx_buf.append(0)    
        elif lvdsCfg_dataFmt_buf[0] == 0: #no ADC no HW
            ADC_file_sessionFlag = "SW"
            CC9_file_sessionFlag = "HW"
            if lvdsCfg_userBufEn_buf[0] == 1:
                ADC_file_numSubframes = ADC_file_numSubframes + 1
                ADC_file_subframeIdx_buf.append(0) 
            else:
                returen_value = TC_FAIL
                print ("Error: subframe 0 has no HW and SW")
        else:
            print ("subframe %d has a invalid dataFmt config" % subframeIdx)
    else:
        returen_value = TC_FAIL
        print ("Error: Invalid lvdsCfg_headerEn_buf[0]")
        
    # Rest of 3 subframes if advanced subframe case 
    for subframeIdx in range (1, CFG_PARAMS['numSubframes']):
        if lvdsCfg_dataFmt_buf[subframeIdx] == 1 or lvdsCfg_dataFmt_buf[subframeIdx] == 4: # 1:ADC 4:CP+ADC+CQ
            if ADC_file_sessionFlag == "HW":
                ADC_file_numSubframes = ADC_file_numSubframes + 1
                ADC_file_subframeIdx_buf.append(subframeIdx) 
            if CC9_file_sessionFlag == "HW":
                CC9_file_numSubframes = CC9_file_numSubframes + 1
                CC9_file_subframeIdx_buf.append(subframeIdx) 
        if lvdsCfg_userBufEn_buf[subframeIdx] == 1:
            if ADC_file_sessionFlag == "SW": 
                ADC_file_numSubframes = ADC_file_numSubframes + 1
                ADC_file_subframeIdx_buf.append(subframeIdx)         
            if CC9_file_sessionFlag == "SW":
                CC9_file_numSubframes = CC9_file_numSubframes + 1
                CC9_file_subframeIdx_buf.append(subframeIdx) 
    
    #print (CFG_PARAMS)
    #print (numOfChirps_buf)
    #print (numLoops_buf)
    #print (numAdcSamples_buf)
    #print (profileIdx_buf)
    #print (SigImgNumSlices_buf)
    #print (RxSatNumSlices_buf)
    #print (chanIdx_buf)

    #print (lvdsCfg_headerEn_buf)
    #print (lvdsCfg_dataFmt_buf)
    #print (lvdsCfg_userBufEn_buf)

    #print (Raw_file_numSubframes)
    #print (Raw_file_subframeIdx_buf)
    #print (Raw_file_sessionFlag)

    #print (ADC_file_numSubframes)
    #print (ADC_file_subframeIdx_buf)
    #print (ADC_file_sessionFlag)

    #print (CC9_file_numSubframes)
    #print (CC9_file_subframeIdx_buf)
    #print (CC9_file_sessionFlag)


def parser_HW_file(capturedFileName, numSubframes, subframeIdx_buf):
    """!
       This function is called by parser_file() to parser lvds demo HW session record file. 

        @param capturedFileName : the record file name to be parsered
        @param numSubframes     : number of subframes on which has HW session
        @param subframeIdx_buf  : buffer to hold the subframe index on which has HW session 
       
        @return return_value : 0 pass otherwise fail
        @return all_frames_ADC_buffer[frame][chirp][rxChan][ADC sample] holds all frames's ADC samples of the entire captured file
        @return all_frames_CP_buffer [frame][chirp][CP]                 holds all frames's CP of the entire captured file
        @return all_frames_CQ1_buffer[frame][chirp][CQ1]                holds all frames's CQ1 of the entire captured file
        @return all_frames_CQ2_buffer[frame][chirp][CQ2]                holds all frames's CQ2 of the entire captured file
    """
    global CFG_PARAMS

    global numOfChirps_buf
    global numLoops_buf
    global numAdcSamples_buf
    global profileIdx_buf
    global SigImgNumSlices_buf
    global RxSatNumSlices_buf
    global chanIdx_buf
    global lvdsCfg_headerEn_buf
    global lvdsCfg_dataFmt_buf
    global lvdsCfg_userBufEn_buf

    #print (capturedFileName)
    #print (numSubframes)
    #print (subframeIdx_buf) 

    orig_stdout = sys.stdout
    fp_out = open(capturedFileName.replace(".bin", "_parser.txt", 1),'w')
    sys.stdout = fp_out  

    if "0ADC" in capturedFileName: 
        ID = 926064613602757340 # 0xCDA 0ADC 0CDA 0ADC
    elif "0CC9" in capturedFileName:
        ID = 705953299182652617 # 0x9CC 0CC9 09CC 0CC9
    else:
        ID = 0 

    ####################################
    ## Parser captured lvds data file ##
    ####################################
    fp = open(capturedFileName,'rb')

    numBytesCapturedFile = os.path.getsize(capturedFileName)

    all_frames_ADC_buffer = []
    all_frames_CP_buffer = []
    all_frames_CQ1_buffer = []
    all_frames_CQ2_buffer = []

    chirpMode = CFG_PARAMS['chirpMode']
    numRxChan = CFG_PARAMS['numRxChan'] 

    return_value = TC_PASS

    frameIdx = 0

    while (1):
        print("\n****** HW session Frame %d ******" %(frameIdx))

        print ("frameIdx = %d" % (int(frameIdx/numSubframes)))

        subframeIdx = subframeIdx_buf[frameIdx%numSubframes]
        print ("subframeIdx = %d" % (subframeIdx)) 
 
        frame_ADC_buffer = []
        frame_CP_buffer = []
        frame_CQ1_buffer = []
        frame_CQ2_buffer = []

        # get current frame's parameters 
        numAdcSamples    = numAdcSamples_buf[subframeIdx]
        profileIdx       = profileIdx_buf[subframeIdx]
        numChirpPerFrame = numLoops_buf[subframeIdx]*numOfChirps_buf[subframeIdx]

        CFG_PARAMS['headerEn'] = lvdsCfg_headerEn_buf[subframeIdx]
        CFG_PARAMS['dataFmt'] = lvdsCfg_dataFmt_buf[subframeIdx]  
                         
        if lvdsCfg_dataFmt_buf[subframeIdx] == 4: # 4:CP+ADC+CQ
            SigImgNumSlices = SigImgNumSlices_buf[subframeIdx]
            RxSatNumSlices  = RxSatNumSlices_buf[subframeIdx]
            
        for groupIdx in range (int(numChirpPerFrame/chirpMode)):
            
            #print("\n****** HW session Frame %d Group %d ******\n" %(frameIdx, groupIdx))
            
            if lvdsCfg_headerEn_buf[subframeIdx] == 1: 
                get_hsi_header(fp)
                return_value += verify_hsi_header_hw(numAdcSamples, ID)
                                
            if '16' in CFG_PARAMS['platfrom'] or '18' in CFG_PARAMS['platfrom'] or '68' in CFG_PARAMS['platfrom'] or '64' in CFG_PARAMS['platfrom']:
                if chirpMode > 1:#multi chirp
 
                    chirp_ADC_buffer = []
                    chirp_CP_buffer = []
                    chirp_CQ1_buffer = []
                    chirp_CQ2_buffer = []

                    if lvdsCfg_dataFmt_buf[subframeIdx] == 1: # 1:ADC
                        for idx in range (numRxChan*chirpMode):  
                            get_ADC(fp, numAdcSamples, CFG_PARAMS['dataSize'])
                            chirp_ADC_buffer.append(ADC_buffer)

                    if lvdsCfg_dataFmt_buf[subframeIdx] == 4: # 4:CP+ADC+CQ
                        for chirpIdx in range (chirpMode): 
                            for chanIdx in range (numRxChan): 
                                (CP_verify_result, CP_buffer) = get_verify_CP(fp, (profileIdx<<2)+chanIdx_buf[chanIdx], groupIdx*chirpMode+chirpIdx)
                                return_value += CP_verify_result
                                chirp_CP_buffer.append(CP_buffer)

                        for idx in range (numRxChan*chirpMode):  
                            get_ADC(fp, numAdcSamples, CFG_PARAMS['dataSize'])
                            chirp_ADC_buffer.append(ADC_buffer)           
                        
                        (CQ_verify_result, CQ1_buffer, CQ2_buffer) = get_verify_CQ(fp, SigImgNumSlices, RxSatNumSlices)
                        return_value += CQ_verify_result
                        chirp_CQ1_buffer.append(CQ1_buffer)
                        chirp_CQ2_buffer.append(CQ2_buffer)
                    
                    frame_ADC_buffer.append(chirp_ADC_buffer)
                    frame_CP_buffer.append(chirp_CP_buffer)
                    frame_CQ1_buffer.append(chirp_CQ1_buffer)
                    frame_CQ2_buffer.append(chirp_CQ2_buffer)

                #end of multi chirp
                else:#single chirp

                    chirp_ADC_buffer = []
                    chirp_CP_buffer = []
                    chirp_CQ1_buffer = []
                    chirp_CQ2_buffer = []

                    if lvdsCfg_dataFmt_buf[subframeIdx] == 1: # 1:ADC
                        for chanIdx in range (numRxChan): 
                            get_ADC(fp, numAdcSamples, CFG_PARAMS['dataSize'])
                            chirp_ADC_buffer.append(ADC_buffer)
                    if lvdsCfg_dataFmt_buf[subframeIdx] == 4: # 4:CP+ADC+CQ
                        for chanIdx in range (numRxChan): 
                            (CP_verify_result, CP_buffer) = get_verify_CP(fp, (profileIdx<<2)+chanIdx_buf[chanIdx], groupIdx)
                            return_value += CP_verify_result
                            chirp_CP_buffer.append(CP_buffer)

                            get_ADC(fp, numAdcSamples, CFG_PARAMS['dataSize'])
                            chirp_ADC_buffer.append(ADC_buffer)

                        (CQ_verify_result, CQ1_buffer, CQ2_buffer) = get_verify_CQ(fp, SigImgNumSlices, RxSatNumSlices)
                        return_value += CQ_verify_result
                        chirp_CQ1_buffer.append(CQ1_buffer)
                        chirp_CQ2_buffer.append(CQ2_buffer)

                    frame_ADC_buffer.append(chirp_ADC_buffer)
                    frame_CP_buffer.append(chirp_CP_buffer)
                    frame_CQ1_buffer.append(chirp_CQ1_buffer)
                    frame_CQ2_buffer.append(chirp_CQ2_buffer)

            #end of 16xx 18xx 68xx 64xx 
        #end for loops
        
        all_frames_ADC_buffer.append(frame_ADC_buffer)  
        all_frames_CP_buffer.append(frame_CP_buffer)  
        all_frames_CQ1_buffer.append(frame_CQ1_buffer)  
        all_frames_CQ2_buffer.append(frame_CQ2_buffer)  
               
        pos = fp.tell()
        if frameIdx == 0:
            numBytesPerFrame = pos
        print("Frame %d end at file location: %d" % (frameIdx, pos))
        
        if pos + numBytesPerFrame > numBytesCapturedFile :
            break

        frameIdx = frameIdx + 1

    # end of frame/while(1) 

    fp.close()

    print ("\n%s contains %d bytes. %d bytes/%d frames has been parsered.\n" % (capturedFileName, numBytesCapturedFile, pos, frameIdx))

    if return_value == TC_PASS:
        print ("\nCaptured file is correct!\n")
    else:
        return_value == TC_FAIL
        print ("\nCaptured file has error!\n")                   
    # end of process for HW session  
    
    sys.stdout = orig_stdout 
    fp_out.close()

    return (return_value, all_frames_ADC_buffer, all_frames_CP_buffer, all_frames_CQ1_buffer, all_frames_CQ2_buffer)

def parser_SW_file(capturedFileName, numFramesToBePrint, numSubframes, subframeIdx_buf):
    """!
       This function is called by parser_file() to parser lvds demo SW session record file

        @param capturedFileName   : the record file name to be parsered 
        @param numFramesToBePrint : number of frames to be print the result on parser log file
        @param numSubframes       : number of subframes on which has SW session
        @param subframeIdx_buf    : buffer to hold the subframe index on which has SW session 
            
        @return return_value : 0 pass otherwise fail
        @return all_frames_x_buffer      [frame][x]             holds all frames's mmw demo detected target's x of the entire captured file
        @return all_frames_y_buffer      [frame][y]             holds all frames's mmw demo detected target's y of the entire captured file
        @return all_frames_z_buffer      [frame][z]             holds all frames's mmw demo detected target's z of the entire captured file
        @return all_frames_v_buffer      [frame][v]             holds all frames's mmw demo detected target's v of the entire captured file
        @return all_frames_range_buffer  [frame][range profile] holds all frames's mmw demo detected range profile
        @return all_frames_azimuth_buffer[frame][azimuth]       holds all frames's mmw demo detected azimuth
        @return all_frames_elevAngle_buffer[frame][elevAngle]   holds all frames's mmw demo detected elevation angle
        @return all_frames_snr_buffer    [frame][snr]           holds all frames's mmw demo detected target's snr of the entire captured file
        @return all_frames_noise_buffer  [frame][noise]         holds all frames's mmw demo detected target's noise of the entire captured file
    """
    global CFG_PARAMS
 
    print (capturedFileName)
    print (numFramesToBePrint)
    print (numSubframes)
    print (subframeIdx_buf)

    orig_stdout = sys.stdout
    fp_out = open(capturedFileName.replace(".bin", "_parser.txt", 1),'w')
    sys.stdout = fp_out  

    if "0ADC" in capturedFileName: 
        ID = 926064613602757340 # 0xCDA 0ADC 0CDA 0ADC
    elif "0CC9" in capturedFileName:
        ID = 705953299182652617 # 0x9CC 0CC9 09CC 0CC9
    else:
        ID = 0 

    PI = 3.14159265

    ####################################
    ## Parser captured lvds data file ##
    ####################################
    fp = open(capturedFileName,'rb')

    numBytesCapturedFile = os.path.getsize(capturedFileName)

    all_frames_numDetObj = []

    all_frames_x_buffer = []

    all_frames_y_buffer = []

    all_frames_z_buffer = []
    
    all_frames_v_buffer = []
    
    all_frames_range_buffer = []
    
    all_frames_azimuth_buffer = []

    all_frames_elevAngle_buffer = []

    all_frames_snr_buffer = []

    all_frames_noise_buffer = []

    return_value = TC_PASS

    frameIdx = 0

    while (1):
        print("\n****** SW session Frame %d ******" %(frameIdx))

        print ("frameIdx = %d" % (int(frameIdx/numSubframes)))

        subframeIdx = subframeIdx_buf[frameIdx%numSubframes]
        print ("subframeIdx = %d" % (subframeIdx)) 
        
        get_hsi_header(fp)
        return_value += verify_hsi_header_sw(fp, ID, int(frameIdx/numSubframes), subframeIdx)
                        
        if '16' in CFG_PARAMS['platfrom'] or '18' in CFG_PARAMS['platfrom'] or '68' in CFG_PARAMS['platfrom'] or '64' in CFG_PARAMS['platfrom']:

            #print(userBuf0)
            #print(userBuf1)
            #print(userBuf2)

            # userBuf0 byte6 and byte7 represent the number of detected objects 
            numDetObj = getUint16(userBuf0[6:8:1]) 

            all_frames_numDetObj.append(numDetObj)
                            
            # Process User Buffer 1
            detectedX_array = []
            detectedY_array = []
            detectedZ_array = []
            detectedV_array = []
            detectedRange_array = []
            detectedAzimuth_array = []
            detectedElevAngle_array = []

            # User Buffer 1 contains x, y, z, v values of all detect objects.
            # each x, y, z, v are 32-bit float in IEEE 754 single-precision binary floating-point format, so every 16 bytes represent x, y, z, v values of one detect objects.    
            
            # for each detect objects, extract/convert float x, y, z, v values and calculate range profile and azimuth 
            for obj in range(numDetObj): 

                # convert byte0 to byte3 to float x value
                x = struct.unpack('<f', codecs.decode(binascii.hexlify(bytearray(userBuf1[obj*16:obj*16+4:1])),'hex'))[0]

                # convert byte4 to byte7 to float y value
                y = struct.unpack('<f', codecs.decode(binascii.hexlify(bytearray(userBuf1[obj*16+4:obj*16+8:1])),'hex'))[0]

                # convert byte8 to byte11 to float z value
                z = struct.unpack('<f', codecs.decode(binascii.hexlify(bytearray(userBuf1[obj*16+8:obj*16+12:1])),'hex'))[0]
                
                # convert byte12 to byte15 to float v value
                v = struct.unpack('<f', codecs.decode(binascii.hexlify(bytearray(userBuf1[obj*16+12:obj*16+16:1])),'hex'))[0]

                # calculate range profile from x, y, z  
                compDetectedRange = math.sqrt((x * x)+(y * y)+(z * z))

                # calculate azimuth from x, y
                if y == 0:
                    if x >= 0:
                        detectedAzimuth = 90
                    else:
                        detectedAzimuth = -90 
                else:
                    detectedAzimuth = math.atan(x/y) * 180 / PI

                # calculate elevation angle from x, y, z
                if x == 0 and y == 0:
                    if z >= 0:
                        detectedElevAngle = 90
                    else: 
                        detectedElevAngle = -90
                else:
                    detectedElevAngle = math.atan(z/math.sqrt((x * x)+(y * y))) * 180 / PI
                            
                detectedX_array.append(x)
                detectedY_array.append(y)
                detectedZ_array.append(z)
                detectedV_array.append(v)
                detectedRange_array.append(compDetectedRange)
                detectedAzimuth_array.append(detectedAzimuth)
                detectedElevAngle_array.append(detectedElevAngle)

            # Process User Buffer 2
            detectedSNR_array = []
            detectedNoise_array = []
            
            # User Buffer 2 contains snr and noise of all detect objects.
            # each snr and noise are 16-bit integer represented by 2 bytes, so every 4 bytes represent snr and noise of one detect objects.    
            
            # for each detect objects, extract snr and noise
            for obj in range(numDetObj):
                # byte0 and byte1 represent snr. convert 2 bytes to 16-bit integer
                snr   = getUint16(userBuf2[(obj*4 + 0):(obj*4 + 2):1]) 
                # byte2 and byte3 represent noise. convert 2 bytes to 16-bit integer
                noise = getUint16(userBuf2[(obj*4 + 2):(obj*4 + 4):1]) 
                detectedSNR_array.append(snr)
                detectedNoise_array.append(noise)
            
            all_frames_x_buffer.append(detectedX_array)
            all_frames_y_buffer.append(detectedY_array)
            all_frames_z_buffer.append(detectedZ_array)
            all_frames_v_buffer.append(detectedV_array)
            all_frames_range_buffer.append(detectedRange_array)
            all_frames_azimuth_buffer.append(detectedAzimuth_array)
            all_frames_elevAngle_buffer.append(detectedElevAngle_array)
            all_frames_snr_buffer.append(detectedSNR_array)
            all_frames_noise_buffer.append(detectedNoise_array)

            if frameIdx < numFramesToBePrint: 
                print("                    x(m)         y(m)         z(m)       v(m/s)    Com0range(m)  azimuth(deg)   elevAngle(deg)   snr(0.1dB)   noise(0.1dB)")
                for obj in range(numDetObj):
                    print("    obj%3d: %12f %12f %12f %12f %12f %14f %12d %12d %14d" % (obj, detectedX_array[obj], detectedY_array[obj], detectedZ_array[obj], detectedV_array[obj], detectedRange_array[obj], detectedAzimuth_array[obj], detectedElevAngle_array[obj], detectedSNR_array[obj], detectedNoise_array[obj]))

        # end of if '16''18''68''64' in CFG_PARAMS['platfrom']: 
        
        pos = fp.tell()
        if frameIdx == 0:
            numBytesPerFrame = pos
        print("Frame %d end at file location: %d\n" % (frameIdx, pos))
        
        if pos + numBytesPerFrame > numBytesCapturedFile :
            break

        frameIdx = frameIdx + 1

    # end of frame/while(1) 

    fp.close()

    print ("\n%s contains %d bytes. %d bytes/%d frames has been parsered.\n" % (capturedFileName, numBytesCapturedFile, pos, frameIdx))

    if return_value == TC_PASS:
        print ("\nCaptured file is correct!\n")
    else:
        return_value == TC_FAIL
        print ("\nCaptured file has error!\n")                     
    
    sys.stdout = orig_stdout 
    fp_out.close()

    return (return_value, all_frames_numDetObj, all_frames_x_buffer, all_frames_y_buffer, all_frames_z_buffer, all_frames_v_buffer, all_frames_range_buffer, all_frames_azimuth_buffer, all_frames_elevAngle_buffer, all_frames_snr_buffer, all_frames_noise_buffer)


def parser_file(config_file_name, numFramesToBePrint, datacard_prefix):
    """!
       This function is called by application to parser lvds demo datacard record files. It reads the entire file and parses all the available frames.
        
        @param config_file_name    : test config profile file name
        @param numFramesToBePrint  : number of frames to be print the result on parser log file
        @param datacard_prefix     : prefix of datacard captured file name

        @return HW_result          : HW session parser result. 0 pass otherwise fail 
        @return HW_numsubframes    : number of subframes on which has HW session
        @return HW_subframeIdx_buf : buffer to hold the subframe index on which has HW session
        @return HW_ADC_buffer      [frame][chirp][rxChan][ADC sample] holds all frames's ADC samples of the entire captured file
        @return HW_CP_buffer       [frame][chirp][CP]                 holds all frames's CP of the entire captured file
        @return HW_CQ1_buffer      [frame][chirp][CQ1]                holds all frames's CQ1 of the entire captured file
        @return HW_CQ2_buffer      [frame][chirp][CQ2]                holds all frames's CQ2 of the entire captured file

        @return SW_result          : SW session parser result. 0 pass otherwise fail
        @return SW_numsubframes    : number of subframes on which has SW session
        @return SW_subframeIdx_buf : buffer to hold the subframe index on which has SW session
        @return SW_numDetObj       : number of the detected objects
        @return SW_x_buffer        [frame][x]                         holds all frames's mmw demo detected target's x of the entire captured file
        @return SW_y_buffer        [frame][y]                         holds all frames's mmw demo detected target's y of the entire captured file
        @return SW_z_buffer        [frame][z]                         holds all frames's mmw demo detected target's z of the entire captured file
        @return SW_v_buffer        [frame][v]                         holds all frames's mmw demo detected target's v of the entire captured file
        @return SW_range_buffer    [frame][range profile]             holds all frames's mmw demo detected range profile
        @return SW_azimuth_buffer  [frame][azimuth]                   holds all frames's mmw demo detected azimuth
        @return SW_elevAngle_buffer[frame][elevAngle]                 holds all frames's mmw demo detected elevation angle
        @return SW_snr_buffer      [frame][snr]                       holds all frames's mmw demo detected target's snr of the entire captured file
        @return SW_noise_buffer    [frame][noise]                     holds all frames's mmw demo detected target's noise of the entire captured file
    """
    global Raw_file_numSubframes
    global Raw_file_subframeIdx_buf
    global Raw_file_sessionFlag

    global ADC_file_numSubframes
    global ADC_file_subframeIdx_buf
    global ADC_file_sessionFlag

    global CC9_file_numSubframes
    global CC9_file_subframeIdx_buf
    global CC9_file_sessionFlag

    # -1:  no HW/SW session
    #  0: has HW/SW session, result pass
    #  1: has HW/SW session, result fail
    HW_result = NOT_APPLY
    SW_result = NOT_APPLY

    HW_numsubframes = 0
    HW_subframeIdx_buf = []

    SW_numsubframes = 0
    SW_subframeIdx_buf = []

    HW_ADC_buffer = []
    HW_CP_buffer = []
    HW_CQ1_buffer = []
    HW_CQ2_buffer = []

    SW_numDetObj = []
    SW_x_buffer = []
    SW_y_buffer = []
    SW_z_buffer = []
    SW_v_buffer = []
    SW_range_buffer = []
    SW_azimuth_buffer = []
    SW_elevAngle_buffer = []
    SW_snr_buffer = []
    SW_noise_buffer = []

    read_config (config_file_name)

    if Raw_file_numSubframes >= 1:

        capturedFileName = datacard_prefix + "_Raw_0.bin"

        if Raw_file_sessionFlag == "HW": 
            HW_result, HW_ADC_buffer, HW_CP_buffer, HW_CQ1_buffer, HW_CQ2_buffer = parser_HW_file(capturedFileName, Raw_file_numSubframes, Raw_file_subframeIdx_buf)

            if HW_result == TC_PASS:
                print ("\n%s parser pass!\n" % (capturedFileName))
            else:
                print ("\n%s parser fail!\n" % (capturedFileName))

            HW_numsubframes    = Raw_file_numSubframes
            HW_subframeIdx_buf = Raw_file_subframeIdx_buf
        elif Raw_file_sessionFlag == "SW":
            SW_result, SW_numDetObj, SW_x_buffer, SW_y_buffer, SW_z_buffer, SW_v_buffer, SW_range_buffer, SW_azimuth_buffer, SW_elevAngle_buffer, SW_snr_buffer, SW_noise_buffer = parser_SW_file(capturedFileName, numFramesToBePrint, Raw_file_numSubframes, Raw_file_subframeIdx_buf)

            if SW_result == TC_PASS:
                print ("\n%s parser pass!\n" % (capturedFileName))
            else:
                print ("\n%s parser fail!\n" % (capturedFileName))
        
            SW_numsubframes    = Raw_file_numSubframes
            SW_subframeIdx_buf = Raw_file_subframeIdx_buf
        else:
            HW_result = TC_FAIL
            SW_result = TC_FAIL
            print ("\nError: Raw file has no session flag!\n")  

    if ADC_file_numSubframes >= 1:

        capturedFileName = datacard_prefix + "_hdr_0ADC_0.bin"

        if ADC_file_sessionFlag == "HW": 
            HW_result, HW_ADC_buffer, HW_CP_buffer, HW_CQ1_buffer, HW_CQ2_buffer = parser_HW_file(capturedFileName, ADC_file_numSubframes, ADC_file_subframeIdx_buf)

            if HW_result == TC_PASS:
                print ("\n%s parser pass!\n" % (capturedFileName))
            else:
                print ("\n%s parser fail!\n" % (capturedFileName))

            HW_numsubframes    = ADC_file_numSubframes
            HW_subframeIdx_buf = ADC_file_subframeIdx_buf

        elif ADC_file_sessionFlag == "SW":
            SW_result, SW_numDetObj, SW_x_buffer, SW_y_buffer, SW_z_buffer, SW_v_buffer, SW_range_buffer, SW_azimuth_buffer, SW_elevAngle_buffer, SW_snr_buffer, SW_noise_buffer = parser_SW_file(capturedFileName, numFramesToBePrint, ADC_file_numSubframes, ADC_file_subframeIdx_buf)

            if SW_result == TC_PASS:
                print ("\n%s parser pass!\n" % (capturedFileName))
            else:
                print ("\n%s parser fail!\n" % (capturedFileName))

            SW_numsubframes    = ADC_file_numSubframes
            SW_subframeIdx_buf = ADC_file_subframeIdx_buf
        else:
            HW_result = TC_FAIL
            SW_result = TC_FAIL
            print ("\nError: 0ADC file has no session flag!\n") 

    if CC9_file_numSubframes >= 1:

        capturedFileName = datacard_prefix + "_hdr_0CC9_0.bin"

        if CC9_file_sessionFlag == "HW": 
            HW_result, HW_ADC_buffer, HW_CP_buffer, HW_CQ1_buffer, HW_CQ2_buffer = parser_HW_file(capturedFileName, CC9_file_numSubframes, CC9_file_subframeIdx_buf)
    
            if HW_result == TC_PASS:
                print ("\n%s parser pass!\n" % (capturedFileName))
            else:
                print ("\n%s parser fail!\n" % (capturedFileName))

            HW_numsubframes    = CC9_file_numSubframes
            HW_subframeIdx_buf = CC9_file_subframeIdx_buf
        elif CC9_file_sessionFlag == "SW":  
            SW_result, SW_numDetObj, SW_x_buffer, SW_y_buffer, SW_z_buffer, SW_v_buffer, SW_range_buffer, SW_azimuth_buffer, SW_elevAngle_buffer, SW_snr_buffer, SW_noise_buffer = parser_SW_file(capturedFileName, numFramesToBePrint, CC9_file_numSubframes, CC9_file_subframeIdx_buf)

            if SW_result == TC_PASS:
                print ("\n%s parser pass!\n" % (capturedFileName))
            else:
                print ("\n%s parser fail!\n" % (capturedFileName))
            SW_numsubframes    = CC9_file_numSubframes
            SW_subframeIdx_buf = CC9_file_subframeIdx_buf
        else:
            HW_result = TC_FAIL
            SW_result = TC_FAIL
            print ("\nError: 0CC9 file has no session flag!\n") 

    return (HW_result, HW_numsubframes, HW_subframeIdx_buf, HW_ADC_buffer, HW_CP_buffer, HW_CQ1_buffer, HW_CQ2_buffer, SW_result, SW_numsubframes, SW_subframeIdx_buf, SW_numDetObj, SW_x_buffer, SW_y_buffer, SW_z_buffer, SW_v_buffer, SW_range_buffer, SW_azimuth_buffer, SW_elevAngle_buffer, SW_snr_buffer, SW_noise_buffer)

    









