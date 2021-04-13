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
# ****************************************************************************


# ****************************************************************************
# Sample mmW demo UART output parser script - should be invoked using python3
#       ex: python3 mmw_demo_example_script.py <recorded_dat_file_from_Visualizer>.dat
#
# Notes:
#   1. The parser_mmw_demo script will output the text version 
#      of the captured files on stdio. User can redirect that output to a log file, if desired
#   2. This example script also outputs the detected point cloud data in mmw_demo_output.csv 
#      to showcase how to use the output of parser_one_mmw_demo_output_packet
# ****************************************************************************

import os
import sys
# import the parser function 
from parser_mmw_demo import parser_one_mmw_demo_output_packet

##################################################################################
# INPUT CONFIGURATION
##################################################################################
# get the captured file name (obtained from Visualizer via 'Record Start')
if (len(sys.argv) > 1):
    capturedFileName=sys.argv[1]
else:
    print ("Error: provide file name of the saved stream from Visualizer for OOB demo")
    exit()

##################################################################################
# USE parser_mmw_demo SCRIPT TO PARSE ABOVE INPUT FILES
##################################################################################
# Read the entire file 
fp = open(capturedFileName,'rb')
readNumBytes = os.path.getsize(capturedFileName)
print("readNumBytes: ", readNumBytes)
allBinData = fp.read()
print("allBinData: ", allBinData[0], allBinData[1], allBinData[2], allBinData[3])
fp.close()

# init local variables
totalBytesParsed = 0;
numFramesParsed = 0;

# parser_one_mmw_demo_output_packet extracts only one complete frame at a time
# so call this in a loop till end of file
while (totalBytesParsed < readNumBytes):
    
    # parser_one_mmw_demo_output_packet function already prints the
    # parsed data to stdio. So showcasing only saving the data to arrays 
    # here for further custom processing
    parser_result, \
    headerStartIndex,  \
    totalPacketNumBytes, \
    numDetObj,  \
    numTlv,  \
    subFrameNumber,  \
    detectedX_array,  \
    detectedY_array,  \
    detectedZ_array,  \
    detectedV_array,  \
    detectedRange_array,  \
    detectedAzimuth_array,  \
    detectedElevation_array,  \
    detectedSNR_array,  \
    detectedNoise_array = parser_one_mmw_demo_output_packet(allBinData[totalBytesParsed::1], readNumBytes-totalBytesParsed)

    # Check the parser result
    print ("Parser result: ", parser_result)
    if (parser_result == 0): 
        totalBytesParsed += (headerStartIndex+totalPacketNumBytes)    
        numFramesParsed+=1
        print("totalBytesParsed: ", totalBytesParsed)
        ##################################################################################
        # TODO: use the arrays returned by above parser as needed. 
        # For array dimensions, see help(parser_one_mmw_demo_output_packet)
        # help(parser_one_mmw_demo_output_packet)
        ##################################################################################

        
        # For example, dump all S/W objects to a csv file
        import csv
        if (numFramesParsed == 1):
            democsvfile = open('mmw_demo_output.csv', 'w', newline='')                
            demoOutputWriter = csv.writer(democsvfile, delimiter=',',
                                    quotechar='', quoting=csv.QUOTE_NONE)                                    
            demoOutputWriter.writerow(["frame","DetObj#","x","y","z","v","snr","noise"])            
            
        for obj in range(numDetObj):
            demoOutputWriter.writerow([numFramesParsed-1, obj, detectedX_array[obj],\
                                           detectedY_array[obj],\
                                           detectedZ_array[obj],\
                                           detectedV_array[obj],\
                                           detectedSNR_array[obj],\
                                           detectedNoise_array[obj]])

        
    else: 
        # error in parsing; exit the loop
        break

# All processing done; Exit
print("numFramesParsed: ", numFramesParsed)
