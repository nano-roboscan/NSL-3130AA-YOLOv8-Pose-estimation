/*
 * Copyright (c) 2013, NANOSYSTEMS CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#ifdef _WINDOWS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <io.h>
#include <process.h>
#else
#include <sstream> 
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/socket.h>
#include <netinet/tcp.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <sys/time.h> 
#endif

#include "NSL3130AA.h"
#include "NSLFrame.h"

#ifdef HAVE_CV_CUDA
#include <opencv2/cudawarping.hpp>
#endif
#include "timeCheck.h"

using namespace cv;

//#define TOFCAM660_ROTATE_IMAGE_90
//#define ROTATE_IMAGE_ADJUST_ROI

#define __STREAMING_COMMAND__


#define DEFAULT_HDR_MODE	HDR_NONE_MODE
//#define DEFAULT_HDR_MODE	HDR_SPATIAL_MODE
//#define DEFAULT_HDR_MODE	HDR_TEMPORAL_MODE

#define opcode_index		9

#define DEFAULT_DIST_OFFSET_VALUE		0
#define OPCODE_SET_OFFSET				0x14

#define MASK_USED_ADC_OVERFLOW 			0x01
#define MASK_USED_SATURATION 			0x02

#define MASK_DRNU_COMPENSATION			0x01
#define MASK_TEMPERATURE_COMPENSATION 	0x02
#define MASK_GRAYSCALE_COMPENSATION 	0x04
#define MASK_AMBIENT_LIGHT_COMPENSATION	0x08

#define SERIAL_START_MARK				0xFFFFAA55 										///<Start marker for the data (camera to host)
#define SERIAL_END_MARK					0xFFFF55AA										   ///<End marker if no CRC is used
#define BAUDRATE660 					10000000
#define SERIAL_BUFFER_SIZE 				1024

#define DEFAULT_ROI_XMIN	0
#define DEFAULT_ROI_XMAX	319
#define DEFAULT_ROI_YMIN	0
#define DEFAULT_ROI_YMAX	239

#define ADJUST_ROI_XMIN		0
#define ADJUST_ROI_XMAX		319
#define ADJUST_ROI_YMIN		60
#define ADJUST_ROI_YMAX		179

#define USB_CODE_ANSWER		0
#define USB_CODE_DATA		1
#define USB_CODE_TRACE		2
#define USB_CODE_DEBUG		3


#define _unused(x) 			((void)(x))


#define ADD_TOFCAM_BUFF(TOFCAMBUFF, MAX_BUFF_CNT)		do{\
															if( TOFCAMBUFF.overflow != 0){\
																if(TOFCAMBUFF.tail_idx + 1 == MAX_BUFF_CNT)\
																	TOFCAMBUFF.tail_idx = 0;\
																else\
																	TOFCAMBUFF.tail_idx++;\
															}\
															if(TOFCAMBUFF.head_idx + 1 == MAX_BUFF_CNT)\
																TOFCAMBUFF.head_idx = 0;\
															else\
																TOFCAMBUFF.head_idx ++;\
															if( TOFCAMBUFF.tail_idx == TOFCAMBUFF.head_idx )\
																TOFCAMBUFF.overflow = 1;\
															else\
																TOFCAMBUFF.overflow = 0;\
														}while(0)

#define	POP_TOFCAM_BUFF(TOFCAMBUFF, MAX_BUFF_CNT)	do{\
														if(TOFCAMBUFF.tail_idx + 1 == MAX_BUFF_CNT)\
															TOFCAMBUFF.tail_idx = 0;\
														else\
															TOFCAMBUFF.tail_idx ++;\
														TOFCAMBUFF.overflow = 0;\
													}while(0)


#define GET_BUFF_CNT(TOFCAMBUFF, MAX_BUFF_CNT)		(( TOFCAMBUFF.head_idx > TOFCAMBUFF.tail_idx) ? (TOFCAMBUFF.head_idx - TOFCAMBUFF.tail_idx) : (TOFCAMBUFF.head_idx < TOFCAMBUFF.tail_idx) ? (MAX_BUFF_CNT + TOFCAMBUFF.head_idx - TOFCAMBUFF.tail_idx) : (TOFCAMBUFF.overflow != 0) ? MAX_BUFF_CNT : 0)



#define MAX_DISTANCEVALUE 12500

static const double PI = 3.14159265358979323846264338328;
static int maxDistanceValue = 12500;
static int maxAmplitudeValue = 2897;
static int maxValidValue = 15000;

static const int indexAmplitudeFactorColor = NSL3130_NUM_COLORS / maxAmplitudeValue;

static uint8_t initialCode660[][100]={	
//	 | ------- start marker --------| |--- length(opcode+data) -----| |-- opcode--| |------- data feild -----------| |-------- end marker --------|
	// COMMAND_SET_INT_TIMES :: int3d(800),  int3d1(100), int3d2(50), grayint(100)
	{14, 0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x0a ,0x00 ,0x01 ,0x05 ,0xDC ,0x01 ,0xF4 ,0x00 ,0x32 ,0x00 ,0x64 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_HDR :: DEFAULT_HDR_MODE
	,{14, 0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x03 ,0x00 ,0x19 ,DEFAULT_HDR_MODE ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_FILTER:: temper filter(2) 0 : THRESHOLD(2) 200(0xC8) :: all off
	,{14, 0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x11 ,0x00 ,0x16 ,0x00 ,0x00 ,0x00 ,0xc8 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_MODULATION : 0(VALUE_12MHZ)
	,{18, 0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x05 ,0x00 ,0x17 ,0x00 ,0x00 ,0x00 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_ROI :: 320 x 240
	,{14, 0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x0a ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x01 ,0x3f ,0x00 ,0xef ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_ADC_OVERFLOW :: adc overflow off, saturation off
	,{14, 0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x04 ,0x00 ,0x0a ,0x00 ,0x00 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_COMPENSATION :: Drnu(1), Temperature(1), AmbientLight(0), Grayscaled(1)
	,{18, 0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x06 ,0x00 ,0x1c ,0x01 ,0x01 ,0x00 ,0x01 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_MIN_AMPLITUDE :: 50
	,{14, 0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x04 ,0x00 ,0x15 ,0x00 ,0x32 ,0xff ,0xff ,0x55 ,0xaa}
};


bool NSL3130AA::hasValidStartMarking(unsigned char *data, int data_len)
{
    return memcmp(data, START_MARKER, 4) == 0 ? true : false;
}

bool NSL3130AA::hasValidEndMarking(unsigned char * data, int data_len)
{
    return memcmp(data, END_MARKER, 4) == 0 ? true : false;
}

bool NSL3130AA::lengthIsCorrect(unsigned char * data, int data_len)
{
    int32_t expectedPayloadBytes = (data[PAYLOAD_SIZE_INDEX] << 24) + (data[PAYLOAD_SIZE_INDEX+1] << 16) + (data[PAYLOAD_SIZE_INDEX+2] << 8) + (data[PAYLOAD_SIZE_INDEX+3] << 0);

    if ((data_len -  PROTOCOL_OVERHEAD_SIZE) == expectedPayloadBytes)
        return true;

    return false;
}


bool NSL3130AA::isValidData(unsigned char *data, int data_len)
{
    if(data_len < NSL3130_NET_HEADER_SIZE)
        return false;

    if(hasValidStartMarking(data, data_len) == false)
        return false;

    if(hasValidEndMarking(data, data_len) == false)
        return false;

    if(lengthIsCorrect(data, data_len) == false)
        return false;

    return true;
}

uint16_t NSL3130AA::getUint16FromCharBuffer(const char *data)
{
    uint16_t value = static_cast<uint16_t>(((static_cast<uint8_t>(data[0])) << 8) + static_cast<uint8_t>(data[1]));
    return value;
}

int16_t NSL3130AA::getInt16FromCharBuffer(const char *data)
{
    int16_t value = static_cast<int16_t>((static_cast<uint8_t>(data[0]) << 8) + static_cast<uint8_t>(data[1]));
    return value;
}


int NSL3130AA::getCamInfo( uint8_t *data )
{
    tofcamInfo.header.version = data[0];
    tofcamInfo.header.dataType = data[1]<<8|data[2];//getUint16FromCharBuffer(&data[1]);
    tofcamInfo.header.width = data[3]<<8|data[4];//getUint16FromCharBuffer(&data[3]);
    tofcamInfo.header.height = data[5]<<8|data[6];//getUint16FromCharBuffer(&data[5]);
    tofcamInfo.header.roiX0 = data[7]<<8|data[8];//getUint16FromCharBuffer(&data[7]);
    tofcamInfo.header.roiY0 = data[9]<<8|data[10];//getUint16FromCharBuffer(&data[9]);
    tofcamInfo.header.roiX1 = data[11]<<8|data[12];//getUint16FromCharBuffer(&data[11]);
    tofcamInfo.header.roiY1 = data[13]<<8|data[14];//getUint16FromCharBuffer(&data[13]);
    tofcamInfo.header.intTime0 = data[15]<<8|data[16];//getUint16FromCharBuffer(&data[15]);
    tofcamInfo.header.intTime1 = data[17]<<8|data[18];//getUint16FromCharBuffer(&data[17]);
    tofcamInfo.header.intTime2 = data[19]<<8|data[20];//getUint16FromCharBuffer(&data[19]);
    tofcamInfo.header.temperature = data[21]<<8|data[22];//getInt16FromCharBuffer(&data[21]);
    tofcamInfo.header.offset = data[23]<<8|data[24];//getUint16FromCharBuffer(&data[23]);

#if 1
// 	width = 320 height = 240 roiX=0/319 roiY=0/239 intTime0 = 800 intTime1 = 100 intTime2 = 50 temper = 3742 offset = 25 dataType = 0
	if( tofcamInfo.header.width != 320 
		|| tofcamInfo.header.height != 240 
		|| tofcamInfo.header.roiX0 != 0 
		|| tofcamInfo.header.roiX1 != 319 
		|| tofcamInfo.header.roiY0 != 0 
		|| tofcamInfo.header.roiY1 != 239 
		|| tofcamInfo.header.offset != 25)
	{
		printf("ver = %d width = %d height = %d roiX=%d/%d roiY=%d/%d intTime0 = %d intTime1 = %d intTime2 = %d temper = %d offset = %d dataType = %d\n"
			, tofcamInfo.header.version
			, tofcamInfo.header.width
			, tofcamInfo.header.height
			, tofcamInfo.header.roiX0
			, tofcamInfo.header.roiX1
			, tofcamInfo.header.roiY0
			, tofcamInfo.header.roiY1
			, tofcamInfo.header.intTime0
			, tofcamInfo.header.intTime1
			, tofcamInfo.header.intTime2
			, tofcamInfo.header.temperature
			, tofcamInfo.header.offset
			, tofcamInfo.header.dataType );

		if( tofcamInfo.header.version != 2 || tofcamInfo.header.width > 320 || tofcamInfo.header.height > 240 ) return 0;
	}
#endif

	return 1;
}

uint16_t NSL3130AA::getHeaderUint16(uint8_t *pData, const int offset)
{
    uint16_t value0 = (unsigned char)(pData[offset]) & 0xFF;
    uint16_t value1 = (unsigned char)(pData[offset+1]) & 0xFF;

    uint16_t value = (value0 << 8) | value1;

    return value;
}

uint32_t NSL3130AA::getHeaderUint32(uint8_t *pData, const int offset)
{
    uint32_t value0 = (unsigned char)(pData[offset]) & 0xFF;
    uint32_t value1 = (unsigned char)(pData[offset+1]) & 0xFF;
    uint32_t value2 = (unsigned char)(pData[offset+2]) & 0xFF;
    uint32_t value3 = (unsigned char)(pData[offset+3]) & 0xFF;

    uint32_t value = (value0 << 24) | (value1 << 16) | (value2 << 8) | value3;

    return value;
}




int NSL3130AA::processUpdData(uint8_t *tempBuffer, int recvedLen, int resp_idx)
{
	uint16_t number = tempBuffer[0]<<8|tempBuffer[1];//getHeaderUint16(tempBuffer, 0);
	uint32_t totalSize = tempBuffer[2]<<24|tempBuffer[3]<<16|tempBuffer[4]<<8|tempBuffer[5];//getHeaderUint32(tempBuffer, 2);
	uint16_t payloadSize = tempBuffer[6]<<8|tempBuffer[7];//getHeaderUint16(tempBuffer, 6);
	uint32_t offset = tempBuffer[8]<<24|tempBuffer[9]<<16|tempBuffer[10]<<8|tempBuffer[11];//getHeaderUint32(tempBuffer, 8);
	uint32_t numPacket = tempBuffer[12]<<24|tempBuffer[13]<<16|tempBuffer[14]<<8|tempBuffer[15];//getHeaderUint32(tempBuffer, 12);
	uint32_t packetNumber = tempBuffer[16]<<24|tempBuffer[17]<<16|tempBuffer[18]<<8|tempBuffer[19];//getHeaderUint32(tempBuffer, 16);
	
	//A new data number is received, so clear anything
	if (number != tofcamInfo.actualNumber)
	{
		if( tofcamInfo.actualNumber != -1 ){
			char dbgbuff[100];
			sprintf(dbgbuff,"actual = %d number = %d/%d packetNumber = %d\n", tofcamInfo.actualNumber, number, numPacket, packetNumber);
			puts(dbgbuff);
		}
		tofcamInfo.actualNumber = number;
		tofcamInfo.receivedBytes = 0;
	}

	//Store the received frame at the required offset
	memcpy(&response[resp_idx][tofcamInfo.receivedBytes], &tempBuffer[UDP_PACKET_HEADER_SIZE], payloadSize);
	tofcamInfo.receivedBytes += payloadSize;
	
#if 0
	printf("number = %d/%d total = %d payload = %d offset = %d numPack = %d packNum = %d recLen = %d/%d\n"
		, number 
		, tofcamInfo.actualNumber 
		, totalSize
		, payloadSize
		, offset
		, numPacket
		, packetNumber
		, recvedLen
		, tofcamInfo.receivedBytes);		
#endif
	//If the last packet is received, the whole data
	if ( packetNumber == (numPacket - 1))
	{
		if( totalSize != tofcamInfo.receivedBytes ){
			tofcamInfo.actualNumber = -1;
			return -1;
		}

		//printf("last number = %d\n", (numPacket - 1)); // 219
		return tofcamInfo.receivedBytes;
	}

	return 0;
}


int NSL3130AA::recvFromTcp(SOCKET sock, uint8_t *total_buf)
{
	static char read_buf[1024];
	uint16_t read_total_size = 0;
	int nbyte = 1;
	int ntotal_length = 0; // start : 4, length 4, end : 4
	struct timeval timeout;    
	fd_set readfds;

	FD_ZERO(&readfds);
	FD_SET(sock, &readfds);
	timeout.tv_sec = 3;
	timeout.tv_usec = 0;  

#ifdef _WINDOWS		
	int state = select(0, &readfds, NULL, NULL, &timeout);
#else
	int state = select(sock+1, &readfds, NULL, NULL, &timeout);
#endif
	if( state == 0 || state == -1 ){  //timeout , error
		printf("recvFromTcp TCP no response state = %d\n", state);
		return 0;
	}
	
	while( nbyte > 0 )
	{
		nbyte = recv( sock, (char*)read_buf, sizeof(read_buf), 0);
		if( nbyte > 0 ){
			memcpy(&total_buf[read_total_size] , read_buf , nbyte);
			read_total_size += nbyte;

			if( read_total_size >= PACKET_INFO_SIZE && ntotal_length == 0 ){ // start prefix(4) + length(4)
				ntotal_length = getHeaderUint32(total_buf, 4);
				ntotal_length += (PACKET_INFO_SIZE + TAIL_PREFIX_SIZE); // ntotal_length = start prefix(4) + length(4) + total_length + end prefix(4) ;
#if 0
				printf("read_total_size = %d ntotal_length = %d nbyte = %d\n", read_total_size, ntotal_length, nbyte);
				if( ntotal_length == 10 ){
					for(int i = 0;i<nbyte;i++){
						printf("%02x ", total_buf[i]);
					}
					printf("\n");
				}
#endif				
			}

			if( read_total_size >= ntotal_length ){
				//printf("read ok : [%d:%d]\n", read_total_size, ntotal_length );
				break;
			}
			else{
				//printf("ing...[%d/%d]\n", nbyte, grayscale_idx );
			}
		}
		else{
			printf("read... 0\n");
			read_total_size = 0;
		}
	}

	return read_total_size;
}


void NSL3130AA::setGrayScaledColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{
	if (value == NSL3130_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == NSL3130_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > end_range)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 255;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else{
		int color = (int)(value * (255.0 /end_range));
		imageLidar.at<Vec3b>(y, x)[0] = color;
		imageLidar.at<Vec3b>(y, x)[1] = color;
		imageLidar.at<Vec3b>(y, x)[2] = color; 
	}

}


int NSL3130AA::setDistanceColor(cv::Mat &imageLidar, int x, int y, int value )
{
	if( value == NSL3130_LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == NSL3130_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if(value == NSL3130_INTERFERENCE)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_EDGE_DETECTED)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_BAD)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
//		imageLidar.at<Vec3b>(y, x)[0] = 255;
//		imageLidar.at<Vec3b>(y, x)[1] = 255;
//		imageLidar.at<Vec3b>(y, x)[2] = 255; 
		imageLidar.at<Vec3b>(y, x) = colorVector.at(colorVector.size()-1);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > maxDistanceValue)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = (int)colorVector.size() - (value*(NSL3130_NUM_COLORS / maxDistanceValue));
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = (int)colorVector.size()-1;
		}
		else if( index > (int)colorVector.size() ){
			index = 0;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

	return value;
}

void NSL3130AA::setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value )
{
	if( value == NSL3130_LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == NSL3130_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if(value == NSL3130_INTERFERENCE)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_EDGE_DETECTED)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == NSL3130_BAD)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<Vec3b>(y, x) = colorVector.at(0);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > maxAmplitudeValue)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = value * indexAmplitudeFactorColor - 1;
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = 0;
		}
		else if( index > (int)colorVector.size() ){
			index = (int)colorVector.size()-1;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

}

int NSL3130AA::getDistanceAmplitude(cv::Mat &imageDistance, cv::Mat &imageAmplitude, bool bUsedPointCloud)
{
	int x, y, index = 0;
	int stepY = 1;
	int maxHeight = tofcamInfo.header.height;
	int maxWidth = tofcamInfo.header.width;
	//uint16_t *pixelPtr = (uint16_t *)&procBuff[1][tofcamInfo.header.offset];

	if( tofcamInfo.config.hdr_mode == HDR_SPATIAL_MODE 
		&& tofcamInfo.header.height <= (NSL3130_IMAGE_HEIGHT>>1) )
	{
		stepY = 2;
		maxHeight <<= 1;
	}

	memset(&catesianTable, 0, sizeof(catesianTable));

//	printf("width = %d height = %d/%d hdr = %d\r\n", tofcamInfo.header.width, tofcamInfo.header.height, maxHeight, tofcamInfo.config.hdr_mode);
#ifdef _WINDOWS
	point_cloud_ptr->clear();
#endif

    for (y = 0; y < maxHeight; y += stepY)
	{
		for (x = 0; x < maxWidth; x++)
		{
			int pixelDistance = (procBuff[1][4*index+1+tofcamInfo.header.offset] << 8) + procBuff[1][4*index+0+tofcamInfo.header.offset];
			int pixelAmplitude = (procBuff[1][4*index+3+tofcamInfo.header.offset] << 8) + procBuff[1][4*index+2+tofcamInfo.header.offset];
			double outX = 0.0f, outY = 0.0f, outZ = 0.0f;

			if( pixelDistance < NSL3130_LIMIT_FOR_VALID_DATA ){
				lensTransform.transformPixel(tofcamInfo.config.roi_xMin+x, tofcamInfo.config.roi_yMin+y, pixelDistance, outX, outY, outZ, sin_angle, cos_angle);
				pixelDistance = outZ;
				catesianTable.x_pos[y*NSL3130_IMAGE_WIDTH+x] = outX;
				catesianTable.y_pos[y*NSL3130_IMAGE_WIDTH+x] = outY;
				catesianTable.z_pos[y*NSL3130_IMAGE_WIDTH+x] = outZ;
			}
			else{
				catesianTable.x_pos[y*NSL3130_IMAGE_WIDTH+x] = 0;
				catesianTable.y_pos[y*NSL3130_IMAGE_WIDTH+x] = 0;
				catesianTable.z_pos[y*NSL3130_IMAGE_WIDTH+x] = 0;
			}

			setDistanceColor(imageDistance, x, y, pixelDistance);
			if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE ) 
				setGrayScaledColor(imageAmplitude, x, y, pixelAmplitude, maxAmplitudeValue);
			else if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
				setGrayScaledColor(imageAmplitude, x, y, pixelAmplitude, 2048.0);
			else 
				setAmplitudeColor(imageAmplitude, x, y, pixelAmplitude);

			if(stepY==2){
				catesianTable.z_pos[(y+1)*NSL3130_IMAGE_WIDTH+x] = pixelDistance;

				if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE ) 
					setGrayScaledColor(imageAmplitude, x, y+1, pixelAmplitude, maxAmplitudeValue);
				else if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
					setGrayScaledColor(imageAmplitude, x, y+1, pixelAmplitude, 2048.0);
				else 
					setAmplitudeColor(imageAmplitude, x, y+1, pixelAmplitude);

				setDistanceColor(imageDistance, x, y+1, pixelDistance);
			}
			
#ifdef _WINDOWS
			if( bUsedPointCloud ){
				pcl::PointXYZRGB point;
				pcl::PointXYZRGB basePoint;
				
				point.x = (double)(outX/1000);
				point.y = (double)(outY/1000);
				point.z = (double)(outZ/1000);

				point.b = imageDistance.at<Vec3b>(y, x)[0];
				point.g = imageDistance.at<Vec3b>(y, x)[1];
				point.r = imageDistance.at<Vec3b>(y, x)[2];

				if(y == 120 || x == 160)
				{ 
					basePoint.x = (double)(outX/1000);
					basePoint.y = (double)(outY/1000);
					basePoint.z = (double)(outZ/1000);

					basePoint.b = 255;
					basePoint.g = 255;
					basePoint.r = 255;
				}

				point_cloud_ptr->points.push_back(point);
				point_cloud_ptr->points.push_back(basePoint);
			}
#endif

			index++;
		}
	}

//	printf("maxAmplitude = %d\n", maxAmplitude);
	return 0;
}


int NSL3130AA::getGrayscaled(cv::Mat &imageLidar, bool bUsedPointCloud)
{
	int index = 0;
	int maxHeight = tofcamInfo.header.height;
	int maxWidth = tofcamInfo.header.width;
	int stepY = 1;
	
	if( tofcamInfo.config.hdr_mode == HDR_SPATIAL_MODE 
		&& tofcamInfo.header.height <= (NSL3130_IMAGE_HEIGHT>>1) )
	{
		stepY = 2;
		maxHeight <<= 1;
	}

	memset(&catesianTable, 0, sizeof(catesianTable));
	//	printf("width = %d height = %d/%d hdr = %d\r\n", tofcamInfo.header.width, tofcamInfo.header.height, maxHeight, tofcamInfo.config.hdr_mode);
#ifdef _WINDOWS
	point_cloud_ptr->clear();
#endif

	for(int y = 0; y < maxHeight; y+=stepY)
	{
		for(int x = 0; x < maxWidth; x++)
		{
			int pixelGrayscale = ( procBuff[1][2*index+1+tofcamInfo.header.offset] << 8) +  procBuff[1][2*index+0+tofcamInfo.header.offset];		
			double outX = 0.0f, outY = 0.0f, outZ = 0.0f;
			
			if( tofcamInfo.tofcamModeType == GRAYSCALE_MODE )
				setGrayScaledColor( imageLidar, x, y, pixelGrayscale, 2048.0);
			else{ // DISTANCE_MODE

				if( pixelGrayscale < NSL3130_LIMIT_FOR_VALID_DATA ){
					lensTransform.transformPixel(tofcamInfo.config.roi_xMin+x, tofcamInfo.config.roi_yMin+y, pixelGrayscale, outX, outY, outZ, sin_angle, cos_angle);
					pixelGrayscale = outZ;
					catesianTable.x_pos[y*NSL3130_IMAGE_WIDTH+x] = outX;
					catesianTable.y_pos[y*NSL3130_IMAGE_WIDTH+x] = outY;
					catesianTable.z_pos[y*NSL3130_IMAGE_WIDTH+x] = outZ;
				}
				else{
					catesianTable.x_pos[y*NSL3130_IMAGE_WIDTH+x] = 0;
					catesianTable.y_pos[y*NSL3130_IMAGE_WIDTH+x] = 0;
					catesianTable.z_pos[y*NSL3130_IMAGE_WIDTH+x] = 0;
				}

				setDistanceColor(imageLidar, x, y, pixelGrayscale);
			}

			if(stepY==2){
				if( tofcamInfo.tofcamModeType == GRAYSCALE_MODE )
					setGrayScaledColor( imageLidar, x, y+1, pixelGrayscale, 2048.0);
				else{
					catesianTable.z_pos[(y+1)*NSL3130_IMAGE_WIDTH+x] = pixelGrayscale;
					setDistanceColor(imageLidar, x, y+1, pixelGrayscale);
				}
			}

			if( tofcamInfo.tofcamModeType != GRAYSCALE_MODE ){
#ifdef _WINDOWS
				if( bUsedPointCloud ){
					pcl::PointXYZRGB point;
					pcl::PointXYZRGB basePoint;

					
					point.x = (double)(outX/1000);
					point.y = (double)(outY/1000);
					point.z = (double)(outZ/1000);

					point.b = imageLidar.at<Vec3b>(y, x)[0];
					point.g = imageLidar.at<Vec3b>(y, x)[1];
					point.r = imageLidar.at<Vec3b>(y, x)[2];

					if(y == 120 || x == 160)
					{ 
						basePoint.x = (double)(outX/1000);
						basePoint.y = (double)(outY/1000);
						basePoint.z = (double)(outZ/1000);

						basePoint.b = 255;
						basePoint.g = 255;
						basePoint.r = 255;
					}

					point_cloud_ptr->points.push_back(point);
					point_cloud_ptr->points.push_back(basePoint);
				}
#endif
			}
			
			index++;
		}
	}

	return 0;
}


uint8_t NSL3130AA::getCommandByType( int modeType )
{
	uint8_t cmd = 0;

	switch(modeType){
		case AMPLITEDE_DISTANCE_MODE:
		case AMPLITEDE_DISTANCE_EX_MODE:
			cmd = 0x02;
			break;
		case DISTANCE_MODE:
			cmd = 0x03;
			break;
		case DISTANCE_GRAYSCALE_MODE:
			cmd = 0x08;
			break;
		case GRAYSCALE_MODE:
			cmd = 0x05;
		default:
			break;
	}
	
	return cmd;
}

int NSL3130AA::sendToDev(SOCKET sock, uint8_t *pData, int nLen, int expectedLen)
{
	static uint8_t serialData[SERIAL_BUFFER_SIZE];
	int pktLen = 0;
	
#ifndef _WINDOWS
	if( ttySerial ){
		uint32_t pyalodLen = nLen;
		
		serialData[0] = (uint8_t)((SERIAL_START_MARK >> 24) & 0xFF);
		serialData[1] = (uint8_t)((SERIAL_START_MARK >> 16) & 0xFF);
		serialData[2] = (uint8_t)((SERIAL_START_MARK >> 8) & 0xFF);
		serialData[3] = (uint8_t)((SERIAL_START_MARK >> 0) & 0xFF);
		
		serialData[4] = (uint8_t)((pyalodLen >> 24) & 0xFF);
		serialData[5] = (uint8_t)((pyalodLen >> 16) & 0xFF);
		serialData[6] = (uint8_t)((pyalodLen >> 8) & 0xFF);
		serialData[7] = (uint8_t)((pyalodLen >> 0) & 0xFF);
		
		memcpy(&serialData[8], pData, nLen);
		
		serialData[SERIAL_BUFFER_SIZE-4] = (uint8_t)((SERIAL_END_MARK >> 24) & 0xFF);
		serialData[SERIAL_BUFFER_SIZE-3] = (uint8_t)((SERIAL_END_MARK >> 16) & 0xFF);
		serialData[SERIAL_BUFFER_SIZE-2] = (uint8_t)((SERIAL_END_MARK >> 8) & 0xFF);
		serialData[SERIAL_BUFFER_SIZE-1] = (uint8_t)((SERIAL_END_MARK >> 0) & 0xFF);

		int ret = write(sock, (const void *)serialData, SERIAL_BUFFER_SIZE);
		_unused(ret);
		if( expectedLen > 0 ) pktLen = rxSerial(serialData, expectedLen, false);
	}
	else
#endif
	{
		int nTotalLen = 0;
		memcpy(&serialData[0], START_MARKER, 4); 
		nTotalLen+=4;

		serialData[nTotalLen+0] = (uint8_t)((nLen >> 24) & 0xff);
		serialData[nTotalLen+1] = (uint8_t)((nLen >> 16) & 0xff);
		serialData[nTotalLen+2] = (uint8_t)((nLen >> 8) & 0xff);
		serialData[nTotalLen+3] = (uint8_t)((nLen >> 0) & 0xff);
		nTotalLen+=4;

		memcpy(&serialData[nTotalLen], pData, nLen);
		nTotalLen+=nLen;

		memcpy(&serialData[nTotalLen], END_MARKER, 4); 
		nTotalLen+=4;

		int ret = send(sock, (char *)serialData, nTotalLen, 0);
		_unused(ret);
		pktLen = recvFromTcp(sock, serialData);
	}

	return pktLen;
}

void NSL3130AA::reqOverflow(SOCKET control_sock)
{
	uint8_t data[10] = {0x00, 0x0a, 0x00, 0x00};
	uint32_t data_len = 4;

	data[2] = (tofcamInfo.config.saturatedFlag & MASK_USED_ADC_OVERFLOW) ? 1 : 0;
	data[3] = (tofcamInfo.config.saturatedFlag & MASK_USED_SATURATION) ? 1 : 0;

	int bComplete = sendToDev(control_sock, data, data_len, 14);

	printf("reqOverflow : read data complete = %d \n", bComplete);
}


void NSL3130AA::reqHdrMode(SOCKET control_sock)
{
	uint8_t data[] = {0x00, 0x19, 0x00};
	uint32_t data_len = 3;	

	data[2] = tofcamInfo.config.hdr_mode;
	
	int bComplete = sendToDev(control_sock, data, data_len, 14);

	printf("reqHdrMode : mode = %d int3d=%d, hdr1=%d, hdr2=%d gray=%d\n", tofcamInfo.config.hdr_mode, tofcamInfo.config.integrationTime3D, tofcamInfo.config.integrationTime3DHdr1, tofcamInfo.config.integrationTime3DHdr2, tofcamInfo.config.integrationTimeGrayScale);
}


void NSL3130AA::reqDualBeam(int control_sock)
{
	// COMMAND_SET_DUALBEAM_MODE :: 3Mhz, off, off
	uint8_t data[] = {0x00 ,0x3e ,0x02 ,0x00 ,0x00};
	uint32_t data_len = 5;	

	data[2] = (tofcamInfo.config.dualbeamState&0xFF);
	
	int bComplete = sendToDev(control_sock, data, data_len, 14);
	(void)bComplete;

	
	printf("reqDualBeam : %d\n", tofcamInfo.config.dualbeamState);
}

void NSL3130AA::reqDeepLearning(int control_sock)
{
	uint8_t data[] = {0x00, 0x0E, 0x00};
	uint32_t data_len = 3;	

	data[2] = tofcamInfo.config.deeplearning;
	
	int bComplete = sendToDev(control_sock, data, data_len, 14);
	(void)bComplete;
	printf("reqDeepLearning : mode = %d\n", tofcamInfo.config.deeplearning);
}



void NSL3130AA::reqCompensation(SOCKET control_sock)
{
	uint8_t data[6] = {0x00, 0x1c, 0x00, 0x00, 0x00, 0x00}; // get grayscale
	uint32_t data_len = 6;	

	data[2] = (tofcamInfo.config.compensationFlag & MASK_DRNU_COMPENSATION) ? 1 : 0;
	data[3] = (tofcamInfo.config.compensationFlag & MASK_TEMPERATURE_COMPENSATION) ? 1 : 0;
	data[4] = (tofcamInfo.config.compensationFlag & MASK_GRAYSCALE_COMPENSATION) ? 1 : 0;
	data[5] = (tofcamInfo.config.compensationFlag & MASK_AMBIENT_LIGHT_COMPENSATION) ? 1 : 0;
	
	int bComplete = sendToDev(control_sock, data, data_len, 18);
	printf("reqCompensation : read data complete = %d, comp = %x \n", bComplete, tofcamInfo.config.compensationFlag);
}



int NSL3130AA::reqStreamingFrame(SOCKET control_sock)
{
	uint8_t data[10] = {0x00, 0x02, VALUE_STREAMING_MEASUREMENT}; // get distance & amplitude
	uint32_t data_len = 3;	
	uint8_t cmdType = getCommandByType(tofcamInfo.tofcamModeType);	

	data[1] = cmdType;

	int bComplete = sendToDev(control_sock, data, data_len, 0);
	printf("reqStreamingFrame : read data complete = %d \n", bComplete);

	return bComplete;
}



void NSL3130AA::reqSingleFrame(SOCKET control_sock, int modeType)
{
	uint8_t data[3] = {0x00, 0x02, VALUE_AUTO_REPEAT_MEASUREMENT};		// USB 24 fps
//	uint8_t data[3] = {0x00, 0x02, VALUE_SINGLE_MEASUREMENT};			// USB 10 fps
	uint32_t data_len = 3;	
	int cmdType = getCommandByType(modeType);	

	data[1] = cmdType;
	
	int bComplete = sendToDev(control_sock, data, data_len, 0);

//	printf("reqSingleFrame : modeType = %d \n", modeType);


}


void NSL3130AA::reqStopStream(SOCKET control_sock)
{
	uint8_t data[2] = {0x00, 0x06};
	uint32_t data_len = 2;	

	int bComplete = sendToDev(control_sock, data, data_len, 14);
	printf("reqStopStream : read data complete = %d \n", bComplete);
}

void NSL3130AA::reqIntegrationTime(SOCKET control_sock)
{
	uint8_t data[] = {0x00 ,0x01 ,0x00 ,0x64 ,0x03 ,0xe8 ,0x00 ,0x00 ,0x55 ,0xf0};
	uint32_t data_len = 10;	

	data[2] = (tofcamInfo.config.integrationTime3D >> 8) & 0xFF;
	data[3] = tofcamInfo.config.integrationTime3D & 0xFF;

	data[4] = (tofcamInfo.config.integrationTime3DHdr1 >> 8) & 0xFF;
	data[5] = tofcamInfo.config.integrationTime3DHdr1 & 0xFF;

	data[6] = (tofcamInfo.config.integrationTime3DHdr2 >> 8) & 0xFF;
	data[7] = tofcamInfo.config.integrationTime3DHdr2 & 0xFF;

	data[8] = (tofcamInfo.config.integrationTimeGrayScale >> 8) & 0xFF;
	data[9] = tofcamInfo.config.integrationTimeGrayScale & 0xFF;

	int bComplete = sendToDev(control_sock, data, data_len, 14);
	printf("setIntegrationTime3D : %d rx = %d\n", tofcamInfo.config.integrationTime3D, bComplete);

}


void NSL3130AA::reqFilterParameter(SOCKET control_sock)
{
	uint8_t data[] = {0x00 ,0x16 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};
	uint32_t data_len = 17;	

	data[2] = (tofcamInfo.config.temporalFilterFactorActual>>8)&0xFF;
	data[3] = (tofcamInfo.config.temporalFilterFactorActual>>0)&0xFF;
	data[4] = (tofcamInfo.config.temporalFilterThreshold>>8)&0xFF;
	data[5] = (tofcamInfo.config.temporalFilterThreshold>>0)&0xFF;
	data[6] = tofcamInfo.config.medianFilterEnable ? 1 : 0;
	data[7] = tofcamInfo.config.averageFilterEnable ? 1 : 0;
	data[8] = (tofcamInfo.config.edgeFilterThreshold>>8)&0xFF;
	data[9] = (tofcamInfo.config.edgeFilterThreshold>>0)&0xFF;
	data[10] = tofcamInfo.config.interferenceUseLashValueEnable ? 1 : 0;
	data[11] = (tofcamInfo.config.interferenceLimit>>8)&0xFF;
	data[12] = (tofcamInfo.config.interferenceLimit>>0)&0xFF;
	data[13] = 0;//(camInfo.config.edgefilterThresholdLow>>8)&0xFF;
	data[14] = 0;//(camInfo.config.edgefilterThresholdLow>>0)&0xFF;
	data[15] = 0;//(camInfo.config.edgefilterThresholdHigh>>8)&0xFF;
	data[16] = 0;//(camInfo.config.edgefilterThresholdHigh>>0)&0xFF;

	int bComplete = sendToDev(control_sock, data, data_len, 14);
	printf("reqFilterParameter : %d\n", bComplete);

}


void NSL3130AA::reqMinAmplitude(SOCKET control_sock)
{
	uint8_t data[] = {0x00 ,0x15 ,0x00 ,0x1e};
	uint32_t data_len = 4;	

	data[2] = (tofcamInfo.config.minAmplitude>>8)&0xFF;
	data[3] = (tofcamInfo.config.minAmplitude>>0)&0xFF;
	
	int bComplete = sendToDev(control_sock, data, data_len, 14);
	printf("reqMinAmplitude : %d\n", tofcamInfo.config.minAmplitude);

}

void NSL3130AA::reqSetROI(SOCKET control_sock)
{
	uint8_t data[] = {0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x01 ,0x3f ,0x00 ,0xef};
	uint32_t data_len = 10;	

#ifdef ROTATE_IMAGE_ADJUST_ROI
	if( tofcamInfo.rotate_90 != 0 ){
		tofcamInfo.config.roi_xMin = ADJUST_ROI_XMIN;
		tofcamInfo.config.roi_xMax = ADJUST_ROI_XMAX;
		tofcamInfo.config.roi_yMin = ADJUST_ROI_YMIN;
		tofcamInfo.config.roi_yMax = ADJUST_ROI_YMAX;
	}
	else{
		tofcamInfo.config.roi_xMin = DEFAULT_ROI_XMIN;
		tofcamInfo.config.roi_xMax = DEFAULT_ROI_XMAX;
		tofcamInfo.config.roi_yMin = DEFAULT_ROI_YMIN;
		tofcamInfo.config.roi_yMax = DEFAULT_ROI_YMAX;
	}

	data[2] = (tofcamInfo.config.roi_xMin>>8)&0xFF;
	data[3] = (tofcamInfo.config.roi_xMin>>0)&0xFF;

	data[4] = (tofcamInfo.config.roi_yMin>>8)&0xFF;
	data[5] = (tofcamInfo.config.roi_yMin>>0)&0xFF;

	data[6] = (tofcamInfo.config.roi_xMax>>8)&0xFF;
	data[7] = (tofcamInfo.config.roi_xMax>>0)&0xFF;

	data[8] = (tofcamInfo.config.roi_yMax>>8)&0xFF;
	data[9] = (tofcamInfo.config.roi_yMax>>0)&0xFF;

#endif

	int bComplete = sendToDev(control_sock, data, data_len, 14);
	printf("reqSetROI rotate 90 : %d\n", tofcamInfo.rotate_90);
}

void NSL3130AA::reqGrayscaleLedControl(SOCKET control_sock, int ledOnOff)
{
	uint8_t data[] = {0x00 ,0x27 ,0x01};
	uint32_t data_len = 3;	

	tofcamInfo.led_control = ledOnOff;	// ON : 1, OFF : 0

	data[2] = ledOnOff;
	
	int bComplete = sendToDev(control_sock, data, data_len, 14);
	printf("reqGrayscaleLedControl : read data complete = %d bOn = %d\n", bComplete, tofcamInfo.led_control);
}



SOCKET NSL3130AA::InitializeControlsocket(void)
{
	SOCKET control_sock;
	struct sockaddr_in   server_addr;
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(NSL3130_NET_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddr.c_str());

	control_sock = socket(PF_INET, SOCK_STREAM, 0);
	if(-1 == control_sock)
	{
		printf("Can not open socket\n");
		exit( 1);
	}
   
	if(-1 == connect(control_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("Socket not connect\n");
		exit(1);
   	}

	return control_sock;
}

SOCKET NSL3130AA::InitializeDataSocket(void)
{
	SOCKET data_socket;
	struct sockaddr_in si_me;

	data_socket = socket(PF_INET, SOCK_DGRAM, 0);
	if(-1 == data_socket)
	{
		printf("Can not open socket\n");
		exit(1);
	}
	
	
	memset(&si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(45454);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
#ifdef _WINDOWS	
	int ret = bind(data_socket, (SOCKADDR*)&si_me, sizeof(si_me));
	if (ret == SOCKET_ERROR) {
		closesocket(data_socket);
		exit(0);
	}

	int sock_opt = 65535;
	int sock_len = sizeof(sock_opt);
	setsockopt(data_socket, SOL_SOCKET, SO_RCVBUF, (char*)&sock_opt, sock_len);
	printf("UDP socket id = %lld\n", data_socket);
#else
	if( bind(data_socket, (struct sockaddr*)&si_me, sizeof(si_me)) < 0 )
	{
		printf("error udp bind...\n");
		exit(0);
	}


	int sock_opt = 160000;
	socklen_t sock_len = sizeof(sock_opt);	
	setsockopt(data_socket, SOL_SOCKET, SO_RCVBUF, &sock_opt, sock_len);

	printf("UDP socket id = %d\n", data_socket);
#endif

	return data_socket;
}

double NSL3130AA::interpolate( double x, double x0, double y0, double x1, double y1){

    if( x1 == x0 ){
        return y0;
    } else {
        return ((x-x0)*(y1-y0)/(x1-x0) + y0);
    }

}


void NSL3130AA::createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue)
{
    double k = 1;
    double BIT0 = -0.125 * k - 0.25;
    double BIT1 = BIT0 + 0.25 * k;
    double BIT2 = BIT1 + 0.25 * k;
    double BIT3 = BIT2 + 0.25 * k;

    double G0 = BIT1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;

    double R0 = BIT2;
    double R1 = R0 + 0.25 * k;
    double R2 = R1 + 0.25 * k;
    double R3 = R2 + 0.25 * k + 0.25;

    double i = (double)indx/(double)numSteps - 0.25 * k;

    if( i>= R0 && i < R1 ){
        red = (unsigned char)interpolate(i, R0, 0, R1, 255);
    } else if((i >= R1) && (i < R2)){
        red = 255;
    } else if((i >= R2) && (i < R3)) {
        red = (unsigned char)interpolate(i, R2, 255, R3, 0);
    } else {
        red = 0;
    }

    if( i>= G0 && i < G1 ){
        green = (unsigned char)interpolate(i, G0, 0, G1, 255);
    } else if((i>=G1)&&(i<G2)){
        green = 255;
    } else if((i >= G2)&&(i < G3)){
        green = (unsigned char)interpolate(i, G2, 255, G3, 0);
    } else {
        green = 0;
    }


    if( i>= BIT0 && i < BIT1 ){
        blue = (unsigned char)interpolate(i, BIT0, 0, BIT1, 255);
    } else if((i >= BIT1)&&(i < BIT2)){
        blue = 255;
    } else if((i >= BIT2)&&(i < BIT3)) {
        blue = (unsigned char)interpolate(i, BIT2, 255, BIT3, 0);
    } else{
        blue = 0;
    }

}

uint8_t *NSL3130AA::convertParameterFromConfig(uint8_t *pData, int nLen)
{
	switch(pData[opcode_index])
	{
		case COMMAND_SET_INT_TIMES:
			pData[10] = (tofcamInfo.config.integrationTime3D>>8)&0xFF;
			pData[11] = (tofcamInfo.config.integrationTime3D>>0)&0xFF;
			
			pData[12] = (tofcamInfo.config.integrationTime3DHdr1>>8)&0xFF;
			pData[13] = (tofcamInfo.config.integrationTime3DHdr1>>0)&0xFF;
			
			pData[14] = (tofcamInfo.config.integrationTime3DHdr2>>8)&0xFF;
			pData[15] = (tofcamInfo.config.integrationTime3DHdr2>>0)&0xFF;
			
			pData[16] = (tofcamInfo.config.integrationTimeGrayScale>>8)&0xFF;
			pData[17] = (tofcamInfo.config.integrationTimeGrayScale>>0)&0xFF;
			break;
		case COMMAND_SET_HDR:
			pData[10] = tofcamInfo.config.hdr_mode;
			break;
		case COMMAND_SET_MODULATION:
			pData[10] = tofcamInfo.config.mod_frequency;
			pData[11] = tofcamInfo.config.mod_channel;
			pData[12] = tofcamInfo.config.mod_autoChannelEnabled;
			break;
		case COMMAND_SET_ROI:
			pData[10] = (tofcamInfo.config.roi_xMin>>8)&0xFF;
			pData[11] = (tofcamInfo.config.roi_xMin>>0)&0xFF;
			
			pData[12] = (tofcamInfo.config.roi_yMin>>8)&0xFF;
			pData[13] = (tofcamInfo.config.roi_yMin>>0)&0xFF;
			
			pData[14] = (tofcamInfo.config.roi_xMax>>8)&0xFF;
			pData[15] = (tofcamInfo.config.roi_xMax>>0)&0xFF;
			
			pData[16] = (tofcamInfo.config.roi_yMax>>8)&0xFF;
			pData[17] = (tofcamInfo.config.roi_yMax>>0)&0xFF;
			break;
		case COMMAND_SET_ADC_OVERFLOW:
			pData[10] = (tofcamInfo.config.saturatedFlag & MASK_USED_ADC_OVERFLOW) ? 1 : 0;
			pData[11] = (tofcamInfo.config.saturatedFlag & MASK_USED_SATURATION) ? 1 : 0;
			break;
		case COMMAND_SET_COMPENSATION:
			pData[10] = (tofcamInfo.config.compensationFlag & MASK_DRNU_COMPENSATION) ? 1 : 0;
			pData[11] = (tofcamInfo.config.compensationFlag & MASK_TEMPERATURE_COMPENSATION) ? 1 : 0;
			pData[12] = (tofcamInfo.config.compensationFlag & MASK_GRAYSCALE_COMPENSATION) ? 1 : 0;
			pData[13] = (tofcamInfo.config.compensationFlag & MASK_AMBIENT_LIGHT_COMPENSATION) ? 1 : 0;
			break;
		case COMMAND_SET_OFFSET:
			pData[10] = (tofcamInfo.config.drnuOffset[tofcamInfo.config.mod_frequency]>>8)&0xFF;
			pData[11] = (tofcamInfo.config.drnuOffset[tofcamInfo.config.mod_frequency]>>0)&0xFF;
			break;
		case COMMAND_SET_MIN_AMPLITUDE:
			pData[10] = (tofcamInfo.config.minAmplitude>>8)&0xFF;
			pData[11] = (tofcamInfo.config.minAmplitude>>0)&0xFF;
			break;
		case COMMAND_SET_FILTER:
			pData[10] = (tofcamInfo.config.temporalFilterFactorActual>>8)&0xFF;
			pData[11] = (tofcamInfo.config.temporalFilterFactorActual>>0)&0xFF;
			pData[12] = (tofcamInfo.config.temporalFilterThreshold>>8)&0xFF;
			pData[13] = (tofcamInfo.config.temporalFilterThreshold>>0)&0xFF;
			pData[14] = tofcamInfo.config.medianFilterEnable ? 1 : 0;
			pData[15] = tofcamInfo.config.averageFilterEnable ? 1 : 0;
			pData[16] = (tofcamInfo.config.edgeFilterThreshold>>8)&0xFF;
			pData[17] = (tofcamInfo.config.edgeFilterThreshold>>0)&0xFF;
			pData[18] = tofcamInfo.config.interferenceUseLashValueEnable ? 1 : 0;
			pData[19] = (tofcamInfo.config.interferenceLimit>>8)&0xFF;
			pData[20] = (tofcamInfo.config.interferenceLimit>>0)&0xFF;
			pData[21] = 0;//(camInfo.config.edgefilterThresholdLow>>8)&0xFF;
			pData[22] = 0;//(camInfo.config.edgefilterThresholdLow>>0)&0xFF;
			pData[23] = 0;//(camInfo.config.edgefilterThresholdHigh>>8)&0xFF;
			pData[24] = 0;//(camInfo.config.edgefilterThresholdHigh>>0)&0xFF;
			break;
		case COMMAND_STOP_STREAM:
			break;
		default:
			printf("undefined OPCODE = 0x%02X\n", pData[opcode_index]);
			break;
	}

	return pData;
}


void NSL3130AA::initializeTofcam660(SOCKET socket)
{	
	int numSteps = NSL3130_NUM_COLORS;
	unsigned char red, green, blue;

	for(int i=0;  i< numSteps; i++)
	{
	  createColorMapPixel(numSteps, i, red, green, blue);
	  colorVector.push_back(Vec3b(blue, green, red));
	}

#if 1
	int size = sizeof(initialCode660) / sizeof(initialCode660[0]);
	for(int i = 0; i < size ; i++){
		int pktLen = (initialCode660[i][5]<<24 | initialCode660[i][6]<<16 | initialCode660[i][7]<<8 | initialCode660[i][8]);

//		printf(":: tx_len = %d\n", pktLen);
		printf("initicode = %02X retLen = %d ttySerial = %d\n", initialCode660[i][10], pktLen, ttySerial);
		
		int bComplete = sendToDev(socket, &convertParameterFromConfig(&initialCode660[i][1], pktLen)[8], pktLen, initialCode660[i][0]);
	}
#endif
	reqIntegrationTime(socket);
	reqSetROI(socket);
	reqOverflow(socket);

	printf("end initializeTofcam660()\n");
}

void NSL3130AA::rxSocket(uint8_t *socketbuff, int buffLen) 
{
	struct sockaddr_in si_other;
	socklen_t addr_size;
	int nbyte = 0;
	struct timeval timeout;    
	fd_set readfds;
	int totalLen = 0;

	tofcamInfo.actualNumber = -1;
#ifndef __STREAMING_COMMAND__
	reqSingleFrame(tofcamInfo.control_sock, tofcamInfo.tofcamModeType);
#endif

//	const auto& time_cap0 = std::chrono::steady_clock::now();

	do
	{
		FD_ZERO(&readfds);
		FD_SET(tofcamInfo.data_sock, &readfds);
		timeout.tv_sec = 0;
		timeout.tv_usec = 500000;  
		
#ifdef _WINDOWS
		int state = select(0, &readfds, NULL, NULL, &timeout);
#else
		int state = select(tofcamInfo.data_sock+1, &readfds, NULL, NULL, &timeout);
#endif
		if( state == 0 || state == -1 ){  //timeout , error
			printf("rxSock datagrame no response state = %d sock = %d\n", state, tofcamInfo.data_sock);
			return;
		}

		addr_size = sizeof(si_other);
		int data_len = recvfrom(tofcamInfo.data_sock, (char *)socketbuff, 1500, 0, (struct sockaddr*)&si_other, &addr_size);
		if (data_len > 0)
		{
			totalLen = processUpdData((uint8_t*)socketbuff, data_len, 1);
			if( totalLen < 0 ){
#ifndef __STREAMING_COMMAND__
				printf("----- retry reqSingleFrame\n");
				reqSingleFrame(tofcamInfo.control_sock, tofcamInfo.tofcamModeType);
#endif
			}
		}
		else{
			printf("err data_len = %d\n", data_len);
		}
	}while( totalLen <= 0 && !exit_thtread );

	if( !exit_thtread )
	{
		EnterCriticalSection(&tofcamBuff.lock);

		tofcamBuff.bufGrayLen[tofcamBuff.head_idx] = 0; 				
		memcpy(tofcamBuff.tofcamBuf[tofcamBuff.head_idx], response[1], totalLen);
		tofcamBuff.bufLen[tofcamBuff.head_idx] = totalLen;
		ADD_TOFCAM_BUFF(tofcamBuff, NSL3130_ETH_BUFF_SIZE);

		LeaveCriticalSection(&tofcamBuff.lock);
	}
	
}


void NSL3130AA::keyProc()
{
	if( tofcamInfo.tofcamEvent_key != 0 )
	{		
		// black & white (grayscale) mode
		if( tofcamInfo.tofcamEvent_key == 'b' )
		{
			if( tofcamInfo.tofcamModeType != GRAYSCALE_MODE )
			{
				tofcamInfo.tofcamModeType = GRAYSCALE_MODE;
#ifdef __STREAMING_COMMAND__
				reqGrayscaleLedControl(tofcamInfo.control_sock, 0);
				if( ttySerial == false ) reqStreamingFrame(tofcamInfo.control_sock);
#endif
			}
		}
		// distance (+grayscale) mode
		else if( tofcamInfo.tofcamEvent_key == 'd')
		{
			if( tofcamInfo.tofcamModeType != DISTANCE_GRAYSCALE_MODE ){
				tofcamInfo.tofcamModeType = DISTANCE_GRAYSCALE_MODE;
#ifdef __STREAMING_COMMAND__
				reqGrayscaleLedControl(tofcamInfo.control_sock, 1);
				if( ttySerial == false ) reqStreamingFrame(tofcamInfo.control_sock);
#endif
			}
		}
		// amplitude & distance mode(grayscale)
		else if( tofcamInfo.tofcamEvent_key == 'e' )
		{
			if( tofcamInfo.tofcamModeType != AMPLITEDE_DISTANCE_EX_MODE ){
				tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_EX_MODE;
#ifdef __STREAMING_COMMAND__
				reqGrayscaleLedControl(tofcamInfo.control_sock, 0);
				if( ttySerial == false ) reqStreamingFrame(tofcamInfo.control_sock);
#endif
			}
		}
		// amplitude & distance mode
		else if( tofcamInfo.tofcamEvent_key == 'a' )
		{
			if( tofcamInfo.tofcamModeType != AMPLITEDE_DISTANCE_MODE ){
				tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_MODE;
#ifdef __STREAMING_COMMAND__
				reqGrayscaleLedControl(tofcamInfo.control_sock, 0);
				if( ttySerial == false ) reqStreamingFrame(tofcamInfo.control_sock);
#endif
			}
		}
		// hdr off
		else if( tofcamInfo.tofcamEvent_key == '0' )
		{
			tofcamInfo.config.hdr_mode = HDR_NONE_MODE;
			reqHdrMode(tofcamInfo.control_sock);
		}
		// hdr spatial
		else if( tofcamInfo.tofcamEvent_key == '1' )
		{
			tofcamInfo.config.hdr_mode = HDR_SPATIAL_MODE;
			reqHdrMode(tofcamInfo.control_sock);
		}
		// hdr temporal
		else if( tofcamInfo.tofcamEvent_key == '2' )
		{
			tofcamInfo.config.hdr_mode = HDR_TEMPORAL_MODE;
			reqHdrMode(tofcamInfo.control_sock);
		}
		// grayscale corrected
		else if( tofcamInfo.tofcamEvent_key == 'g' )
		{
			if( tofcamInfo.config.compensationFlag & MASK_GRAYSCALE_COMPENSATION )
				tofcamInfo.config.compensationFlag &= ~MASK_GRAYSCALE_COMPENSATION;
			else
				tofcamInfo.config.compensationFlag |= MASK_GRAYSCALE_COMPENSATION;

			reqCompensation(tofcamInfo.control_sock);
		}
		// ambient light enable/disable
		else if( tofcamInfo.tofcamEvent_key == 'l' )
		{
			if( tofcamInfo.config.compensationFlag & MASK_AMBIENT_LIGHT_COMPENSATION )
				tofcamInfo.config.compensationFlag &= ~MASK_AMBIENT_LIGHT_COMPENSATION;
			else
				tofcamInfo.config.compensationFlag |= MASK_AMBIENT_LIGHT_COMPENSATION;
		
			reqCompensation(tofcamInfo.control_sock);
		}
		else if( tofcamInfo.tofcamEvent_key == 'u' )
		{

			if( tofcamInfo.config.compensationFlag & MASK_DRNU_COMPENSATION ){
				tofcamInfo.config.compensationFlag &= ~MASK_DRNU_COMPENSATION;
				tofcamInfo.config.compensationFlag &= ~MASK_TEMPERATURE_COMPENSATION;
			}
			else{
				tofcamInfo.config.compensationFlag |= MASK_DRNU_COMPENSATION;
				tofcamInfo.config.compensationFlag |= MASK_TEMPERATURE_COMPENSATION;
			}

			reqCompensation(tofcamInfo.control_sock);
		}
		// saturation enable/disable
		else if( tofcamInfo.tofcamEvent_key == 's' )
		{
			if( tofcamInfo.config.saturatedFlag & MASK_USED_SATURATION )
				tofcamInfo.config.saturatedFlag &= ~MASK_USED_SATURATION;
			else
				tofcamInfo.config.saturatedFlag |= MASK_USED_SATURATION;

			reqOverflow(tofcamInfo.control_sock);
		}
		// overflow enable/disable
		else if( tofcamInfo.tofcamEvent_key == 'f' )
		{
			if( tofcamInfo.config.saturatedFlag & MASK_USED_ADC_OVERFLOW )
				tofcamInfo.config.saturatedFlag &= ~MASK_USED_ADC_OVERFLOW;
			else
				tofcamInfo.config.saturatedFlag |= MASK_USED_ADC_OVERFLOW;
			
			reqOverflow(tofcamInfo.control_sock);
		}
		else if( tofcamInfo.tofcamEvent_key == 'r' )
		{
			tofcamInfo.rotate_90 = tofcamInfo.rotate_90 ? 0 : 1;
#ifdef ROTATE_IMAGE_ADJUST_ROI
			reqSetROI(tofcamInfo.control_sock);
#endif
		}
		else if ( tofcamInfo.tofcamEvent_key == 't'){
			tofcamInfo.led_control ^= 1;
			reqGrayscaleLedControl(tofcamInfo.control_sock, tofcamInfo.led_control);
		}
		else if ( tofcamInfo.tofcamEvent_key == 'p' ){
			tofcamInfo.usedPointCloud = tofcamInfo.usedPointCloud ? 0 : 1; 
		}
		else if( tofcamInfo.tofcamEvent_key == 'h' ){
			printf("-----------------------------------------------\n");
			printf("p key : change Point Cloud\n");
			printf("b key : change GRAYSCALE mode\n");
			printf("d key : change DISTANCE & Grayscale mode\n");
			printf("a key : change AMPLITUDE & DISTANCE mode\n");
			printf("e key : change AMPLITUDE(log) & DISTANCE mode\n");
			printf("t key : change Grayscale LED\n");
			printf("g key : change Grayscale corrected\n");
			printf("l key : change Ambient light corrected\n");
			printf("s key : change saturation corrected\n");
			printf("f key : change overflow corrected\n");
			printf("u key : change DRNU\n");
			printf("r key : rotate 90(ROI reset)\n");
			printf("0 key : change HDR off\n");
			printf("1 key : change HDR Spatial\n");
			printf("2 key : change HDR Temporal\n");
			printf("-----------------------------------------------\n");

			tofcamInfo.printBasicInfo = tofcamInfo.printBasicInfo ? 0 : 1;
		}

		tofcamInfo.tofcamEvent_key = 0;
	}
}


void *NSL3130AA::rxTofcam660(void *arg) 
{	
	static uint8_t socketbuff[NSL3130_BUFF_SIZE];

	while(!exit_thtread)
	{
		if( tofcamInfo.captureNetType == NONEMODEL_TYPE )
		{
			Sleep(10);
			continue;
		}

		keyProc();

#ifndef _WINDOWS
		if( ttySerial ) {

			int expectedRcvLen = 307238; // 4 * NSL3130_IMAGE_WIDTH * NSL3130_IMAGE_HEIGHT + UART_OVERHEAD_SIZE + DATA_HEADER_SIZE;
			if( tofcamInfo.tofcamModeType == DISTANCE_MODE || tofcamInfo.tofcamModeType == GRAYSCALE_MODE ){
				expectedRcvLen = 2 * NSL3130_IMAGE_WIDTH * NSL3130_IMAGE_HEIGHT + UART_OVERHEAD_SIZE + DATA_HEADER_SIZE;
			}
/*
			else if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_MODE
				|| tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE
				|| tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE ){
				expectedRcvLen = 4 * NSL3130_IMAGE_WIDTH * NSL3130_IMAGE_HEIGHT + UART_OVERHEAD_SIZE + DATA_HEADER_SIZE;
			}
*/			

			reqSingleFrame(tofcamInfo.control_sock, tofcamInfo.tofcamModeType);
			if( rxSerial(socketbuff, expectedRcvLen, true) < 0 ){
				closesocket(tofcamInfo.control_sock);
				tofcamInfo.control_sock = 0;
				setSerialBaudrate();
			}
		}
		else
#endif
		{
			rxSocket(socketbuff, sizeof(socketbuff));
		}

		
		Sleep(1);
	}

	return NULL;
}

//point cloud
#ifdef _WINDOWS
pcl::PointCloud<pcl::PointXYZRGB>::Ptr NSL3130AA::pcbVis()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_ptr->clear();
	point_cloud_ptr->is_dense = false;
	//point_cloud_ptr->reserve(IMAGE_WIDTH * IMAGE_HEIGHT);
	point_cloud_ptr->width = NSL3130_IMAGE_WIDTH;
	point_cloud_ptr->height = NSL3130_IMAGE_HEIGHT;

	return point_cloud_ptr;
}

pcl::visualization::PCLVisualizer::Ptr NSL3130AA::rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ROBOSCAN PointCloud"));
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//ÁÂÇ¥Ãà
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, -5, 0, 0, 0, 0, -1, 0, 0);
	viewer->setShowFPS(false);
	//viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
#if 0
	pcl::PointXYZ a1, a2;
	a1.x = -1.450;
	a1.y = -0.450;
	a1.z = 5.000;
	a2.x = 1.450;
	a2.y = -0.450;
	a2.z = 5.000;
	viewer->addLine(a1, a2, "a12");

	pcl::PointXYZ a3, a4;
	a3.x = -1.450;
	a3.y = 0.450;
	a3.z = 5.000;
	a4.x = 1.450;
	a4.y = 0.450;
	a4.z = 5.000;
	viewer->addLine(a3, a4, "a34");

	pcl::PointXYZ a5, a6;
	a5.x = -1.450;
	a5.y = -0.450;
	a5.z = 3.000;
	a6.x = 1.450;
	a6.y = -0.450;
	a6.z = 3.000;
	viewer->addLine(a5, a6, "a56");

	pcl::PointXYZ a7, a8;
	a7.x = -1.450;
	a7.y = 0.450;
	a7.z = 3.000;
	a8.x = 1.450;
	a8.y = 0.450;
	a8.z = 3.000;
	viewer->addLine(a7, a8, "a78");

	viewer->addLine(a1, a5, "a15");
	viewer->addLine(a2, a6, "a26");
	viewer->addLine(a3, a7, "a37");
	viewer->addLine(a4, a8, "a45");

	viewer->addLine(a1, a3, "a13");
	viewer->addLine(a2, a4, "a24");
	viewer->addLine(a5, a7, "a57");
	viewer->addLine(a6, a8, "a68");
#endif
	return (viewer);
}

#else

int NSL3130AA::setSerialBaudrate(void)
{
	int fileID;

	char path[100];
	sprintf(path, "%s", mIpaddr.c_str());
	fileID = open(path, O_RDWR | O_NOCTTY | O_SYNC);
	if( fileID < 0 ) return 0;
	tcflush(fileID, TCIOFLUSH);

	struct termios tty;
	memset (&tty, 0, sizeof tty);

	tcgetattr (fileID, &tty); //TODO...

	cfsetospeed (&tty, B4000000);
	cfsetispeed (&tty, B4000000);

	// no canonical processing
	// disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars

	tty.c_oflag = 0;				// no remapping, no delays
//	tty.c_oflag &= ~(ONLCR | OCRNL); //TODO...

	tty.c_lflag = 0;				// no signaling chars, no echo,
//	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); //TODO...

//	tty.c_iflag = (IGNBRK|IGNPAR);
	tty.c_iflag &= ~IGNBRK; 		// disable break processing
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	tty.c_iflag &= ~(INLCR | IGNCR | ICRNL); //TODO...

	tty.c_cc[VMIN]	= 0;			// non-blocking read
	tty.c_cc[VTIME] = 5;			// 0.5 second read timeout

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; 	// 8-bit chars				  
	tty.c_cflag &= ~(PARENB | PARODD);	// shut off parity
	tty.c_cflag &= ~CSTOPB;    //one stop bit
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls   

	tcflush(fileID, TCIOFLUSH);

	if (tcsetattr (fileID, TCSANOW, &tty) != 0){
		printf("error %d from tcsetattr\n", errno);
		return 0;
	}

	printf("opened USB-Serial : %d\n", fileID);
	tofcamInfo.control_sock = fileID;
	flushRx();

	return fileID;
}


int NSL3130AA::flushRx(void)
{
	uint8_t buf[5000];
	int n = 0;
	int readflushData = 0;

	reqSingleFrame(tofcamInfo.control_sock, tofcamInfo.tofcamModeType);

	while(true)
	{
		n = read(tofcamInfo.control_sock, buf, 5000);

		if(n > 0){
			readflushData += n;
		}else if( n == -1 ){
			printf("flush Error on  SerialConnection::readRxData= -1\n");
			break;
		}else if( n == 0 ){
			printf("flush readData %d bytes\n", readflushData);
			break;
		}

	}

	return 0;
}




int NSL3130AA::rxSerial(uint8_t *socketbuff, int buffLen, bool addQue) 
{
    uint8_t buf[4096];
	int n = 0;

//	const auto& time_cap0 = std::chrono::steady_clock::now();

	for(int i=0; i< buffLen; i+=n)
	{
		unsigned long int buf_size = buffLen;
		if(buf_size > sizeof(buf))
			buf_size = sizeof(buf);

		n = read(tofcamInfo.control_sock, buf, buf_size);

		if(n > 0){						  
			memcpy(socketbuff + i, buf, n);
		}else if(n == -1){
			printf("Error on  SerialConnection::readRxData= -1\n");
			return -1;
		}else if(n == 0 && i < buffLen-1){
			printf("serialConnection->readRxData %d bytes from %d received\n", i, buffLen);
			return -2;
		}

	}

//	const auto& time_cap1 = std::chrono::steady_clock::now();
//	double time_cam = (time_cap1 - time_cap0).count() / 1000000.0;
//	printf("  serial-Rx:		   %9.3lf [msec] len = %d\n", time_cam, buffLen);

	if( !exit_thtread && addQue == true )
	{
		unsigned int type = socketbuff[4];

		if( memcmp(socketbuff, START_MARKER, 4) != 0 )
		{
			printf("error start marker [%02X:%02X:%02X:%02X]\n", socketbuff[0], socketbuff[1], socketbuff[2], socketbuff[3]);
		}
		else if( type == 1 ){
			EnterCriticalSection(&tofcamBuff.lock);
			
			tofcamBuff.bufGrayLen[tofcamBuff.head_idx] = 0;
			memcpy(tofcamBuff.tofcamBuf[tofcamBuff.head_idx], &socketbuff[9], buffLen-13);
			tofcamBuff.bufLen[tofcamBuff.head_idx] = buffLen;
			ADD_TOFCAM_BUFF(tofcamBuff, NSL3130_ETH_BUFF_SIZE);
			
			LeaveCriticalSection(&tofcamBuff.lock);
		}
		else{
			printf("err recv data type = %d totalLen = %d recv = %d\n", type, buffLen, tofcamInfo.receivedBytes);
		}

		tofcamInfo.receivedBytes = 0;
	}
	else if( addQue == false ){
		tofcamInfo.receivedBytes = 0;
	}

	return buffLen;
}

#endif


void NSL3130AA::drawHistogram(cv::Mat &bgr_image)
{
	cv::Mat grayImg, hist_img;
	cv::cvtColor(bgr_image, grayImg, cv::COLOR_BGR2GRAY);
	cv::equalizeHist(grayImg, hist_img);
	cv::cvtColor(hist_img, bgr_image, cv::COLOR_GRAY2BGR);

#if 0
	cv::Mat lab_image;
	cv::cvtColor(bgr_image, lab_image, cv::COLOR_BGR2Lab);

	// Extract the L channel
	std::vector<cv::Mat> lab_planes(3);
	cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]
	
	// apply the CLAHE algorithm to the L channel
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
	clahe->setClipLimit(4);
//	clahe->setTilesGridSize(cv::Size(8, 8));
	cv::Mat dst;
	clahe->apply(lab_planes[0], dst);
	
	// Merge the the color planes back into an Lab image
	dst.copyTo(lab_planes[0]);
	cv::merge(lab_planes, lab_image);
	
	// convert back to RGB
	cv::cvtColor(lab_image, bgr_image, cv::COLOR_Lab2BGR);
#endif
}

//////////////////////////////////// External Interface ////////////////////////////////////////

// Create
NSL3130AA* NSL3130AA::Create( std::string ipaddr )
{
	// create camera instance
	return new NSL3130AA(ipaddr);
}


int NSL3130AA::getVideoWidth(){
	if( tofcamInfo.rotate_90 != 0 ){
		return NSL3130_IMAGE_HEIGHT;
	}
	return NSL3130_IMAGE_WIDTH;
}

int NSL3130AA::getVideoHeight(){
	if( tofcamInfo.rotate_90 != 0 ){
		return NSL3130_IMAGE_WIDTH;
	}
	return NSL3130_IMAGE_HEIGHT;
}

int NSL3130AA::getWidthDiv()				
{ 
	if( tofcamInfo.rotate_90 != 0 ){
		return MODEL_HEIGHT/NSL3130_IMAGE_HEIGHT;
	}
	return MODEL_WIDTH/NSL3130_IMAGE_WIDTH; 
}

int NSL3130AA::getHeightDiv()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return MODEL_WIDTH/NSL3130_IMAGE_WIDTH;
	}
	return MODEL_HEIGHT/NSL3130_IMAGE_HEIGHT; 
}

int NSL3130AA::getWidth()				
{ 
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.imageHeight;
	}
	return tofcamInfo.imageWidth; 
}

/**
 * Return the height of the stream, in pixels.
 */
int NSL3130AA::getHeight()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.imageWidth;
	}
	return tofcamInfo.imageHeight; 
}

int NSL3130AA::getDLWidth()				
{ 
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.imageHeight;
	}
	return tofcamInfo.imageWidth; 
}

/**
 * Return the height of the stream, in pixels.
 */
int NSL3130AA::getDLHeight()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.imageWidth;
	}
	return tofcamInfo.imageHeight; 
}

void NSL3130AA::setCameraSize(int width, int height)
{
	tofcamInfo.imageWidth = width;
	tofcamInfo.imageHeight = height;
}




bool NSL3130AA::isRotate90()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return true;
	}

	return false;
}

void NSL3130AA::drawPointCloud(void)
{
#ifdef _WINDOWS
	if( !viewer->wasStopped() && point_cloud_ptr->points.size() > 0 ){
		viewer->updatePointCloud(point_cloud_ptr, "sample cloud");
		viewer->spinOnce();
	}
#endif
}

std::string NSL3130AA::getDistanceString(int distance )
{
	std::string distStr;

	if( distance == NSL3130_LOW_AMPLITUDE || distance == 300000 )
		distStr = "LOW_AMPLITUDE";
	else if( distance == NSL3130_ADC_OVERFLOW )
		distStr = "ADC_OVERFLOW";
	else if( distance == NSL3130_SATURATION )
		distStr = "SATURATION";
	else if( distance == NSL3130_INTERFERENCE )
		distStr = "INTERFERENCE";
	else if( distance == NSL3130_EDGE_DETECTED )
		distStr = "EDGE_DETECTED";
	else if( distance == NSL3130_BAD )
		distStr = "BAD_FIXEL";
	else
		distStr = format("%d mm", distance);

	return distStr;
}

std::string NSL3130AA::getLeftViewName(void)
{
	std::string nameStr;
	if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_MODE )
		nameStr = "AMPLITUDE";
	else if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE )
		nameStr = "AMPL&Gray";
	else //if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
		nameStr = "GRAYSCALE";	

	return nameStr;
}


// Capture
bool NSL3130AA::Capture( void** output, int timeout )
{
	// verify the output pointer exists
	if( !output )
		return false;

	TimeCheck tmChk;
	tmChk.setPrint(false);
	tmChk.setBegin();

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	int frame_cnt = 0;
	while(!exit_thtread)
	{
		if( GET_BUFF_CNT(tofcamBuff, NSL3130_ETH_BUFF_SIZE) > 0 ){
			tmChk.setEnd();
			tmChk.printTime("RxTime");
			tmChk.setBegin();

			cv::Mat image(NSL3130_IMAGE_HEIGHT, NSL3130_IMAGE_WIDTH, CV_8UC3, Scalar(255,255,255));	
			cv::Mat imageDist(NSL3130_IMAGE_HEIGHT, NSL3130_IMAGE_WIDTH, CV_8UC3, Scalar(255,255,255));			

			EnterCriticalSection(&tofcamBuff.lock);
			//printf("main-bufLen = %d:%d\n", tofcamBuff.bufGrayLen[tofcamBuff.tail_idx], tofcamBuff.bufLen[tofcamBuff.tail_idx]);
			if( tofcamBuff.bufGrayLen[tofcamBuff.tail_idx] > 0 ) memcpy(procBuff[0], tofcamBuff.tofcamGrayBuf[tofcamBuff.tail_idx], tofcamBuff.bufGrayLen[tofcamBuff.tail_idx]);
			memcpy(procBuff[1], tofcamBuff.tofcamBuf[tofcamBuff.tail_idx], tofcamBuff.bufLen[tofcamBuff.tail_idx]);
			POP_TOFCAM_BUFF(tofcamBuff, NSL3130_ETH_BUFF_SIZE);

			LeaveCriticalSection(&tofcamBuff.lock);


			if( getCamInfo(procBuff[1]) == 0 ) {
				printf("err::version mismatch~~\n");
				continue;
			}

			if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_MODE
				|| tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE
				|| tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE )
			{
				getDistanceAmplitude(imageDist, image, tofcamInfo.usedPointCloud != 0 );
			}
			else{
				getGrayscaled(image, tofcamInfo.usedPointCloud != 0);
				imageDist = image;
			}

			tmChk.setEnd();
			tmChk.printTime("RxConvert");
			tmChk.setBegin();

			static cv::Mat resizeDist, resizeFrame;

			if( tofcamInfo.rotate_90 != 0 )
			{
				cv::rotate(image, image, ROTATE_90_CLOCKWISE);
				cv::rotate(imageDist, imageDist, ROTATE_90_CLOCKWISE);

				cv::resize( image, resizeFrame, cv::Size( tofcamInfo.imageHeight, tofcamInfo.imageWidth ) , cv::INTER_LANCZOS4);
				cv::resize( imageDist, resizeDist, cv::Size( tofcamInfo.imageHeight, tofcamInfo.imageWidth ), cv::INTER_LANCZOS4);
			}
			else{
				cv::resize( image, resizeFrame, cv::Size( tofcamInfo.imageWidth, tofcamInfo.imageHeight ) , cv::INTER_LANCZOS4);
				cv::resize( imageDist, resizeDist, cv::Size( tofcamInfo.imageWidth, tofcamInfo.imageHeight ), cv::INTER_LANCZOS4);
			}

			tmChk.setEnd();
			tmChk.printTime("reSize");
			tmChk.setBegin();

			if( GRAYSCALE_MODE != tofcamInfo.tofcamModeType && 
				AMPLITEDE_DISTANCE_MODE != tofcamInfo.tofcamModeType && 
				DISTANCE_GRAYSCALE_MODE != tofcamInfo.tofcamModeType )
				drawHistogram(resizeFrame);

			tmChk.setEnd();
			tmChk.printTime("histogram");
			
			tofcamImage.frameMat = &resizeFrame;
			tofcamImage.distMat = &resizeDist;		
			tofcamImage.pCatesianTable = &catesianTable;
			tofcamImage.isRotate = tofcamInfo.rotate_90;
			break;

		}
		else{
			Sleep(1);
			
			std::chrono::steady_clock::time_point curTime = std::chrono::steady_clock::now();
			double passed_time = (curTime - begin).count() / 1000000.0;
			if( passed_time > timeout ){
				printf("timeout capture~~~~~~~ timeout = %d\n", timeout);
				return false;
			}

			continue;
		}
	}

	*output = &tofcamImage;
	return true;
}

// closeLidar
void NSL3130AA::closeLidar()
{
	tofcamInfo.captureNetType = NONEMODEL_TYPE;

	if( exit_thtread == 0 ){
	    exit_thtread = 1;

#ifdef _WINDOWS
		DWORD rst = WaitForSingleObject(hThread, 1000);
		if (rst != WAIT_OBJECT_0) {
			printf("wait error\n");
		}

#else
		pthread_join(threadID, NULL);
#endif
		
		reqStopStream(tofcamInfo.control_sock);

		if( tofcamInfo.control_sock != 0 ){
			closesocket(tofcamInfo.control_sock);
			tofcamInfo.control_sock = 0;
		}

		if( tofcamInfo.data_sock != 0 ){
			closesocket(tofcamInfo.data_sock);
			tofcamInfo.data_sock = 0;
		}
	}
}

void NSL3130AA::startCaptureCommand(int netType, CaptureOptions &camOpt )
{
	maxDistanceValue = (camOpt.maxDistance <= 0 || camOpt.maxDistance > MAX_DISTANCEVALUE) ? MAX_DISTANCEVALUE : camOpt.maxDistance;

	tofcamInfo.tofcamModeType = camOpt.captureType;
	tofcamInfo.config.integrationTimeGrayScale = camOpt.grayIntegrationTime;
	tofcamInfo.config.integrationTime3D = camOpt.integrationTime;
	tofcamInfo.config.minAmplitude = camOpt.minAmplitude;
	tofcamInfo.config.dualbeamState = camOpt.dualbeamState;
	tofcamInfo.config.hdr_mode = camOpt.hdr_mode;
	tofcamInfo.config.deeplearning = camOpt.deeplearning;

	tofcamInfo.config.medianFilterEnable = camOpt.medianFilterEnable;
	tofcamInfo.config.edgeFilterThreshold = camOpt.edgeThresHold;
	tofcamInfo.config.averageFilterEnable = camOpt.averageFilterEnable;
	tofcamInfo.config.temporalFilterFactorActual = camOpt.temporalFilterFactorActual;
	tofcamInfo.config.temporalFilterThreshold = camOpt.temporalFilterThreshold;
	tofcamInfo.config.interferenceUseLashValueEnable = camOpt.interferenceUseLashValueEnable;
	tofcamInfo.config.interferenceLimit = camOpt.interferenceLimit;
	
	reqIntegrationTime(tofcamInfo.control_sock);
	reqMinAmplitude(tofcamInfo.control_sock);
	reqFilterParameter(tofcamInfo.control_sock);

	reqHdrMode(tofcamInfo.control_sock);
	reqDualBeam(tofcamInfo.control_sock);
	reqDeepLearning(tofcamInfo.control_sock);

	if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE ){
		reqGrayscaleLedControl(tofcamInfo.control_sock, 1);
	}

	
#ifdef __STREAMING_COMMAND__
	if( ttySerial == false ) reqStreamingFrame(tofcamInfo.control_sock);
#endif	
	tofcamInfo.captureNetType = netType;

	printf("start Capture~~~ intTime = %d/%d captureType =%d\n", camOpt.integrationTime, camOpt.grayIntegrationTime, camOpt.captureType);
	
}

void NSL3130AA::setKey(int cmdKey)
{	
	tofcamInfo.tofcamEvent_key = cmdKey;
}


// constructor
NSL3130AA::NSL3130AA( std::string ipaddr )
{	
	mIpaddr = ipaddr;
	ttySerial = false;


	exit_thtread = 0;
	
	tofcamBuff.overflow = 0;
	tofcamBuff.head_idx = 0;
	tofcamBuff.tail_idx = 0;


	memset(&tofcamInfo, 0, sizeof(tofcamInfo));
//	tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_MODE;
	tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_EX_MODE; // distance & amplitude(grayscale)
//	tofcamInfo.tofcamModeType = DISTANCE_GRAYSCALE_MODE;
//	tofcamInfo.tofcamModeType = GRAYSCALE_MODE;
//	tofcamInfo.tofcamModeType = DISTANCE_MODE;


#ifdef TOFCAM660_ROTATE_IMAGE_90
	tofcamInfo.rotate_90 = 1;
	tofcamInfo.config.roi_xMin = ADJUST_ROI_XMIN;
	tofcamInfo.config.roi_xMax = ADJUST_ROI_XMAX;
	tofcamInfo.config.roi_yMin = ADJUST_ROI_YMIN;
	tofcamInfo.config.roi_yMax = ADJUST_ROI_YMAX;
#else
	tofcamInfo.rotate_90 = 0;
	tofcamInfo.config.roi_xMin = DEFAULT_ROI_XMIN;
	tofcamInfo.config.roi_xMax = DEFAULT_ROI_XMAX;
	tofcamInfo.config.roi_yMin = DEFAULT_ROI_YMIN;
	tofcamInfo.config.roi_yMax = DEFAULT_ROI_YMAX;
#endif


	tofcamInfo.led_control = 1;
	tofcamInfo.captureNetType = NONEMODEL_TYPE;
	tofcamInfo.config.hdr_mode = DEFAULT_HDR_MODE;
	tofcamInfo.config.integrationTime3D = 800;
	tofcamInfo.config.integrationTime3DHdr1 = 100;
	tofcamInfo.config.integrationTime3DHdr2 = 50;
	tofcamInfo.config.integrationTimeGrayScale = 100;
	
//	tofcamInfo.config.saturatedFlag |= MASK_USED_ADC_OVERFLOW;
//	tofcamInfo.config.saturatedFlag |= MASK_USED_SATURATION;
	tofcamInfo.config.compensationFlag |= MASK_DRNU_COMPENSATION;
	tofcamInfo.config.compensationFlag |= MASK_TEMPERATURE_COMPENSATION;
	tofcamInfo.config.compensationFlag |= MASK_GRAYSCALE_COMPENSATION;
//	if( tofcamInfo.config.integrationTimeGrayScale == 0 || tofcamInfo.config.integrationTimeGrayScale == 50 )
//		tofcamInfo.config.compensationFlag |= MASK_AMBIENT_LIGHT_COMPENSATION;
	tofcamInfo.config.minAmplitude = 50;	// 0x32

	tofcamInfo.imageWidth = MODEL_WIDTH;
	tofcamInfo.imageHeight = MODEL_HEIGHT;

	printf("ipaddr = %s\n", mIpaddr.c_str());

#ifndef _WINDOWS
	if( !( mIpaddr.at(0) >= 0x30 && mIpaddr.at(0) <= 0x39 ) ){
		ttySerial = true;
		setSerialBaudrate();
	}
#endif

	if( tofcamInfo.control_sock == 0 ) {
		if( ttySerial ){
			ttySerial = false;
			printf("error ::: USB not opened\n");
			printf("changed ipaddr /dev/ttyLidar -> 192.168.0.220\n");
			mIpaddr = "192.168.0.220";
		}

		tofcamInfo.control_sock = InitializeControlsocket();
		tofcamInfo.data_sock = InitializeDataSocket();
	}

	
	initializeTofcam660(tofcamInfo.control_sock);

	double psdAngle = 0.0f; //seobi psd angle 0'
	sin_angle = sin(psdAngle*PI/180.0);
	cos_angle = cos(psdAngle*PI/180.0);

	lensTransform.initLensDistortionTable(STANDARD_FIELD);	// WIDE_FIELD(110), STANDARD_FIELD(90), NARROW_FIELD(50)

#ifdef _WINDOWS
	unsigned threadID;
	hThread = (HANDLE)_beginthreadex(NULL, 0, &NSL3130AA::rxWrapper, this, 0, &threadID);

	tofcamInfo.usedPointCloud = 1;	
	point_cloud_ptr = pcbVis();
	viewer = rgbVis(point_cloud_ptr);
#else
	tofcamInfo.usedPointCloud = 0;	
	pthread_create(&threadID, NULL, NSL3130AA::rxWrapper, this);
#endif
}


// destructor	
NSL3130AA::~NSL3130AA()
{
	printf("~NSL3130AA\n");
	closeLidar();
	return;
}




